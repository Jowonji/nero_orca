"""Relative wrist-image teleop → Nero 7-DoF damped-least-squares IK.

MediaPipe world landmarks are wrist-relative, so they cannot move the arm in
space. The publisher appends image-space wrist (x, y) and palm width; the first
valid sample is the origin, later samples are a delta around the anchor EE pose.
Orientation is held at the rest end-effector rotation (no camera-to-base calib).

With an Orbbec RGB-D camera the wrist position also carries the palm point
(X, Y, Z) in meters in the color camera frame. Then the delta is metric
(METRIC_SCALE m of EE per m of hand) instead of image-gain based; samples with
missing depth are dropped rather than mixed with the image cue.

Smoothing / robustness:
- One Euro filter on the image cue (low lag on fast moves, heavy smoothing at rest).
- x is scaled by the image aspect so left/right and up/down have the same gain.
- image mode: depth uses origin_palm / palm - 1 (distance ∝ 1 / apparent size).
- EE target speed is capped so a tracking glitch cannot yank the arm.
- nullspace term pulls the redundant DoF toward rest so the elbow stops wandering.
- stale wrist position (hand lost) holds the current pose; re-detection re-anchors without a jump.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from limits import JOINTS, REST_QPOS

EE_BODY = "orca_right_R-Carpals_8d1f1041"
# Image +x (right) → robot +Y; image +y (down) → robot -Z; hand farther → robot +X.
IMAGE_ASPECT = 640.0 / 480.0
LATERAL_GAIN = 0.40  # m per image height
VERTICAL_GAIN = 0.40  # m per image height
DEPTH_GAIN = 0.25  # m per relative distance change (0.5 = hand 50% farther)
# Robot X, Y, Z (m) around rest. ±25/30/25 cm: worst model gravity load 76% (joint 2);
# only the far, raised corners are out of reach (IK stops at the closest pose).
POS_CLAMP = np.array([0.25, 0.30, 0.25], dtype=np.float64)
MAX_EE_SPEED = 0.8  # m/s

# Pick-and-place mode (--pick-place): J3/J5/J6 locked at 0 so J2, J4, J7 share one axis,
# and J7 = PICK_PITCH - J2 - J4 keeps the hand at one tilt (fingers ~45 deg down). J1, J2, J4
# set the hand position: a 3-DoF action that is easier to learn. Fixed tilt shrinks the reach,
# so the home sits mid-workspace in front of the robot instead of at REST_QPOS.
PICK_LOCKED = (2, 4, 5)  # J3, J5, J6 indices
PICK_PITCH = math.radians(135.0)
PICK_HOME_EE = np.array([-0.52, 0.0, 0.23], dtype=np.float64)  # m; robot faces -X
# Box around home (m). Table = robot base plane; the hand reaches 9.8 cm below the EE body
# at this tilt, so z >= 0.12 keeps the fingertips ~2 cm above the table.
PICK_CLAMP_LO = np.array([-0.10, -0.25, -0.11], dtype=np.float64)
PICK_CLAMP_HI = np.array([0.10, 0.25, 0.11], dtype=np.float64)
# Slower target so the recorded action is one the arm actually follows (at 0.8 m/s joint 2
# lagged up to 41 deg and tripped the tracking-error hold).
PICK_MAX_EE_SPEED = 0.3  # m/s
# Elbow stays bent: past the reach boundary the IK otherwise straightens J4 through 0 and
# flips to the elbow-inverted branch (seen on the real arm: J4 -24 deg, J7 pinned at 89).
PICK_J4_MIN = math.radians(15.0)
# Joint speed cap on the command itself, so the recorded action never jumps (leaving the J4
# floor at the reach edge jumped J4 17 deg in one tick) and stays one the arm can follow: at
# 0.6 rad/s with --nero-speed 10 the real J1 managed ~17 deg/s and tripped the 20 deg
# tracking hold 5 times in 36 s. Below hardware.NERO_MAX_JOINT_SPEED (0.6) on purpose.
PICK_MAX_JOINT_SPEED = 0.4  # rad/s; run the arm at --nero-speed 20
WRIST_TIMEOUT = 0.5  # s without a new MediaPipe sample → hold and re-anchor later
MIN_PALM_WIDTH = 0.02

# RGB-D mode: camera frame x right, y down, z away from the camera.
METRIC_SCALE = 1.5  # m of EE motion per m of hand motion (1.0 felt too narrow)

# One Euro filter per channel: x, y, palm width. Palm width is the noisiest.
FILTER_MIN_CUTOFF = np.array([1.2, 1.2, 0.6], dtype=np.float64)  # Hz
FILTER_BETA = np.array([8.0, 8.0, 4.0], dtype=np.float64)
FILTER_D_CUTOFF = 1.0
# RGB-D channels: X, Y, Z (m). Stereo depth Z is noisier than the lateral axes.
METRIC_FILTER_MIN_CUTOFF = np.array([1.2, 1.2, 0.8], dtype=np.float64)  # Hz
METRIC_FILTER_BETA = np.array([15.0, 15.0, 10.0], dtype=np.float64)

DAMPING = 1.5e-3
STEP_CLIP = 0.12
IK_ITERS = 10
NULLSPACE_GAIN = 0.05  # per IK iteration, toward REST_QPOS


@dataclass
class WristPosition:
    """Image-space wrist cue: x, y in [0, 1], palm_width in [0, 1]; optional metric palm."""

    wrist_image: np.ndarray  # (3,)
    stamp: float = field(default_factory=time.perf_counter)
    palm_xyz: np.ndarray | None = None  # (3,) camera frame, meters

    @property
    def metric(self) -> bool:
        return self.palm_xyz is not None

    @property
    def cue(self) -> np.ndarray:
        return self.wrist_image if self.palm_xyz is None else self.palm_xyz


class OneEuroFilter:
    """Vector One Euro filter (Casiez et al. 2012) driven by wall-clock dt."""

    def __init__(self, min_cutoff: np.ndarray, beta: np.ndarray, d_cutoff: float) -> None:
        self._min_cutoff = np.asarray(min_cutoff, dtype=np.float64)
        self._beta = np.asarray(beta, dtype=np.float64)
        self._d_cutoff = float(d_cutoff)
        self._x: np.ndarray | None = None
        self._dx = np.zeros_like(self._min_cutoff)

    @staticmethod
    def _alpha(cutoff: np.ndarray | float, dt: float) -> np.ndarray:
        tau = 1.0 / (2.0 * math.pi * np.asarray(cutoff, dtype=np.float64))
        return 1.0 / (1.0 + tau / dt)

    def reset(self, x: np.ndarray | None = None) -> None:
        self._x = None if x is None else np.array(x, dtype=np.float64)
        self._dx = np.zeros_like(self._min_cutoff)

    def __call__(self, x: np.ndarray, dt: float) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        if self._x is None or dt <= 0.0:
            if self._x is None:
                self._x = x.copy()
            return self._x.copy()
        dx = (x - self._x) / dt
        a_d = self._alpha(self._d_cutoff, dt)
        self._dx = a_d * dx + (1.0 - a_d) * self._dx
        cutoff = self._min_cutoff + self._beta * np.abs(self._dx)
        a = self._alpha(cutoff, dt)
        self._x = a * x + (1.0 - a) * self._x
        return self._x.copy()


class WristPositionMirror:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: WristPosition | None = None

    def update_from_landmarks(self, landmarks: object) -> None:
        wrist = getattr(landmarks, "wrist_position", None)
        if wrist is None:
            return
        arr = np.asarray(wrist, dtype=np.float64).reshape(-1)
        if arr.shape[0] < 3 or not np.all(np.isfinite(arr[:3])):
            return
        if arr[2] < MIN_PALM_WIDTH:
            return
        palm_xyz = None
        if arr.shape[0] >= 6:
            if not np.all(np.isfinite(arr[3:6])):
                return  # RGB-D publisher without depth on the palm: hold, don't mix cues
            palm_xyz = arr[3:6].copy()
        with self._lock:
            self._latest = WristPosition(
                wrist_image=arr[:3].copy(), stamp=time.perf_counter(), palm_xyz=palm_xyz
            )

    def snapshot(self) -> WristPosition | None:
        with self._lock:
            return self._latest

    def reset(self) -> None:
        with self._lock:
            self._latest = None


class NeroArmIK:
    def __init__(self, model, mirror_lateral: bool = False, pick_place: bool = False) -> None:
        """``mirror_lateral``: the camera faces the operator (a mirror view), so the
        operator's right is the image's left; flip robot Y so right still means right.
        ``pick_place``: locked-wrist 3-DoF mode around ``PICK_HOME_EE`` (see PICK_*)."""
        import mujoco

        self._model = model
        self._lateral_sign = -1.0 if mirror_lateral else 1.0
        self._ee = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, EE_BODY)
        if self._ee < 0:
            raise RuntimeError(f"IK end-effector body missing: {EE_BODY}")
        self._lo = np.array([spec["sim_lower"] for spec in JOINTS], dtype=np.float64)
        self._hi = np.array([spec["sim_upper"] for spec in JOINTS], dtype=np.float64)
        self._pick = bool(pick_place)
        self._clamp_lo = PICK_CLAMP_LO if self._pick else -POS_CLAMP
        self._clamp_hi = PICK_CLAMP_HI if self._pick else POS_CLAMP
        self._max_ee_speed = PICK_MAX_EE_SPEED if self._pick else MAX_EE_SPEED
        self._rest_q = self._pick_home_q() if self._pick else np.array(REST_QPOS, dtype=np.float64)
        self._q = self._rest_q.copy()
        self._image_filter = OneEuroFilter(FILTER_MIN_CUTOFF, FILTER_BETA, FILTER_D_CUTOFF)
        self._metric_filter = OneEuroFilter(
            METRIC_FILTER_MIN_CUTOFF, METRIC_FILTER_BETA, FILTER_D_CUTOFF
        )
        self._filter = self._image_filter
        self._metric = False
        self._rest_ee = np.zeros(3, dtype=np.float64)
        self._anchor_ee = np.zeros(3, dtype=np.float64)
        self._target = np.zeros(3, dtype=np.float64)
        self._origin: np.ndarray | None = None
        self._last_solve: float | None = None

    @property
    def home_q(self) -> np.ndarray:
        """Arm pose the teleop starts from: REST_QPOS, or the pick-and-place home."""
        return self._rest_q.copy()

    def _pick_home_q(self) -> np.ndarray:
        import mujoco

        data = mujoco.MjData(self._model)
        self._q = np.array(REST_QPOS, dtype=np.float64)
        self._q[1], self._q[3] = 0.5, 1.5  # elbow-up seed in front of the robot
        for _ in range(40):
            self._q = self._ik_pick(data, PICK_HOME_EE)
        data.qpos[:7] = self._q
        mujoco.mj_kinematics(self._model, data)
        miss = float(np.linalg.norm(PICK_HOME_EE - data.xpos[self._ee]))
        if miss > 0.005:
            raise RuntimeError(f"pick-and-place home unreachable ({miss * 100:.1f} cm off)")
        return self._q.copy()

    def capture_rest_ee(self, data) -> None:
        import mujoco

        data.qpos[:7] = self._rest_q
        mujoco.mj_forward(self._model, data)
        self._rest_ee = np.array(data.xpos[self._ee], dtype=np.float64)
        self._anchor_ee = self._rest_ee.copy()
        self._target = self._rest_ee.copy()
        self._q = self._rest_q.copy()
        self._origin = None
        self._image_filter.reset()
        self._metric_filter.reset()
        self._last_solve = None

    def _offset(self, cue: np.ndarray) -> np.ndarray:
        assert self._origin is not None
        if self._metric:
            d = cue - self._origin
            return METRIC_SCALE * np.array(
                [d[2], self._lateral_sign * d[0], -d[1]], dtype=np.float64
            )
        dx = (cue[0] - self._origin[0]) * IMAGE_ASPECT
        dy = cue[1] - self._origin[1]
        depth = self._origin[2] / max(cue[2], MIN_PALM_WIDTH) - 1.0
        return np.array(
            [DEPTH_GAIN * depth, self._lateral_sign * LATERAL_GAIN * dx, -VERTICAL_GAIN * dy],
            dtype=np.float64,
        )

    def solve(self, data, wrist: WristPosition | None) -> np.ndarray:
        now = time.perf_counter()
        dt = 0.0 if self._last_solve is None else min(now - self._last_solve, 0.2)
        self._last_solve = now

        stale = wrist is None or now - wrist.stamp > WRIST_TIMEOUT
        if stale or (self._origin is not None and wrist.metric != self._metric):
            # Hand lost (or cue type changed): hold, re-anchor on the next detection.
            if self._origin is not None:
                self._origin = None
                self._anchor_ee = self._target.copy()
            if stale:
                return self._q.copy()

        if self._origin is None:
            self._metric = wrist.metric
            self._filter = self._metric_filter if self._metric else self._image_filter
            self._origin = wrist.cue.copy()
            self._filter.reset(wrist.cue)
            return self._q.copy()

        cue = self._filter(wrist.cue, dt)
        offset = self._anchor_ee + self._offset(cue) - self._rest_ee
        offset = np.clip(offset, self._clamp_lo, self._clamp_hi)
        goal = self._rest_ee + offset

        step = goal - self._target
        max_step = self._max_ee_speed * dt
        dist = float(np.linalg.norm(step))
        if dist > max_step > 0.0:
            step *= max_step / dist
        elif max_step <= 0.0:
            step[:] = 0.0
        self._target = self._target + step
        q = self._ik(data, self._target)
        if self._pick:
            q = self._limit_joint_step(q, dt)
        self._q = q
        return self._q.copy()

    def _limit_joint_step(self, q: np.ndarray, dt: float) -> np.ndarray:
        """Scale the whole joint step down uniformly (keeps J7 = pitch - J2 - J4 exact)."""
        step = q - self._q
        ratio = float(np.max(np.abs(step))) / max(PICK_MAX_JOINT_SPEED * dt, 1e-9)
        return self._q + step / ratio if ratio > 1.0 else q

    def _ik(self, data, target: np.ndarray) -> np.ndarray:
        import mujoco

        if self._pick:
            return self._ik_pick(data, target)
        q_save = np.array(data.qpos[:7], dtype=np.float64)
        q = self._q.copy()
        jacp = np.zeros((3, self._model.nv))
        eye7 = np.eye(7)
        try:
            for _ in range(IK_ITERS):
                data.qpos[:7] = q
                mujoco.mj_forward(self._model, data)
                err = target - np.asarray(data.xpos[self._ee], dtype=np.float64)
                mujoco.mj_jacBody(self._model, data, jacp, None, self._ee)
                j = jacp[:, :7]
                j_pinv = j.T @ np.linalg.inv(j @ j.T + DAMPING * np.eye(3))
                dq = j_pinv @ err
                dq += (eye7 - j_pinv @ j) @ (NULLSPACE_GAIN * (self._rest_q - q))
                dq = np.clip(dq, -STEP_CLIP, STEP_CLIP)
                q = np.clip(q + dq, self._lo, self._hi)
                if float(np.linalg.norm(err)) < 1e-4:
                    break
            return q
        except np.linalg.LinAlgError:
            return self._q.copy()
        finally:
            data.qpos[:7] = q_save
            mujoco.mj_forward(self._model, data)

    def _ik_pick(self, data, target: np.ndarray) -> np.ndarray:
        """Position IK over J1, J2, J4 with J3/J5/J6 at 0 and J7 holding the hand tilt."""
        import mujoco

        q_save = np.array(data.qpos[:7], dtype=np.float64)
        q = self._q.copy()
        q[list(PICK_LOCKED)] = 0.0
        act = [0, 1, 3]
        jacp = np.zeros((3, self._model.nv))
        try:
            for _ in range(IK_ITERS):
                q[6] = PICK_PITCH - q[1] - q[3]
                data.qpos[:7] = q
                mujoco.mj_forward(self._model, data)
                err = target - np.asarray(data.xpos[self._ee], dtype=np.float64)
                if float(np.linalg.norm(err)) < 1e-4:
                    break
                mujoco.mj_jacBody(self._model, data, jacp, None, self._ee)
                j = jacp[:, :7].copy()
                j[:, 1] -= j[:, 6]  # J7 moves by -(dJ2 + dJ4)
                j[:, 3] -= j[:, 6]
                ja = j[:, act]
                dq = ja.T @ np.linalg.solve(ja @ ja.T + DAMPING * np.eye(3), err)
                q[act] = np.clip(q[act] + np.clip(dq, -STEP_CLIP, STEP_CLIP), self._lo[act], self._hi[act])
                q[3] = max(q[3], PICK_J4_MIN)
                # J7 = PICK_PITCH - J2 - J4 must stay inside its limits, so bound J2 + J4.
                q[3] = np.clip(q[3], PICK_PITCH - self._hi[6] - q[1], PICK_PITCH - self._lo[6] - q[1])
                if q[3] < PICK_J4_MIN:  # only J2 can restore the bound without flipping
                    q[3] = PICK_J4_MIN
                    q[1] = np.clip(PICK_PITCH - self._hi[6] - q[3], self._lo[1], self._hi[1])
            q[6] = np.clip(PICK_PITCH - q[1] - q[3], self._lo[6], self._hi[6])
            return q
        except np.linalg.LinAlgError:
            return self._q.copy()
        finally:
            data.qpos[:7] = q_save
            mujoco.mj_forward(self._model, data)
