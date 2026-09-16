"""Relative wrist-image teleop → Nero 7-DoF damped-least-squares IK.

MediaPipe world landmarks are wrist-relative, so they cannot move the arm in
space. The publisher appends image-space wrist (x, y) and palm width; the first
valid sample is the origin, later samples are a delta around the rest EE pose.
Orientation is held at the rest end-effector rotation (no camera-to-base calib).
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

import numpy as np

from limits import JOINTS, REST_QPOS

EE_BODY = "orca_right_R-Carpals_8d1f1041"
# Image +x (right) → robot +Y; image +y (down) → robot -Z; larger palm (closer) → robot -X.
POS_GAIN = np.array([-0.80, 0.40, -0.40], dtype=np.float64)
POS_CLAMP = 0.18
DAMPING = 1.5e-3
STEP_CLIP = 0.12
IK_ITERS = 10


@dataclass
class ArmHint:
    """Image-space wrist cue: x, y in [0, 1], palm_width in [0, 1]."""

    wrist_image: np.ndarray  # (3,)


class ArmHintMirror:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: ArmHint | None = None

    def update_from_landmarks(self, landmarks: object) -> None:
        wrist = getattr(landmarks, "wrist_image", None)
        if wrist is None:
            return
        arr = np.asarray(wrist, dtype=np.float64).reshape(-1)
        if arr.shape[0] < 3 or not np.all(np.isfinite(arr[:3])):
            return
        with self._lock:
            self._latest = ArmHint(wrist_image=arr[:3].copy())

    def snapshot(self) -> ArmHint | None:
        with self._lock:
            return self._latest

    def reset(self) -> None:
        with self._lock:
            self._latest = None


class NeroArmIK:
    def __init__(self, model) -> None:
        import mujoco

        self._model = model
        self._ee = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, EE_BODY)
        if self._ee < 0:
            raise RuntimeError(f"IK end-effector body missing: {EE_BODY}")
        self._lo = np.array([spec["sim_lower"] for spec in JOINTS], dtype=np.float64)
        self._hi = np.array([spec["sim_upper"] for spec in JOINTS], dtype=np.float64)
        self._q = np.array(REST_QPOS, dtype=np.float64)
        self._origin: np.ndarray | None = None
        self._rest_ee = np.zeros(3, dtype=np.float64)

    def capture_rest_ee(self, data) -> None:
        import mujoco

        data.qpos[:7] = REST_QPOS
        mujoco.mj_forward(self._model, data)
        self._rest_ee = np.array(data.xpos[self._ee], dtype=np.float64)
        self._q = np.array(REST_QPOS, dtype=np.float64)
        self._origin = None

    def solve(self, data, hint: ArmHint | None) -> np.ndarray:
        import mujoco

        if hint is None:
            self._q = np.array(REST_QPOS, dtype=np.float64)
            return self._q.copy()
        if self._origin is None:
            self._origin = hint.wrist_image.copy()
            return self._q.copy()

        delta = hint.wrist_image - self._origin
        offset = np.array(
            [
                POS_GAIN[0] * delta[2],  # closer (larger palm) → robot -X
                POS_GAIN[1] * delta[0],  # image right → robot +Y
                POS_GAIN[2] * delta[1],  # image down → robot -Z
            ],
            dtype=np.float64,
        )
        offset = np.clip(offset, -POS_CLAMP, POS_CLAMP)
        target = self._rest_ee + offset

        q_save = np.array(data.qpos[:7], dtype=np.float64)
        q = self._q.copy()
        jacp = np.zeros((3, self._model.nv))
        try:
            for _ in range(IK_ITERS):
                data.qpos[:7] = q
                mujoco.mj_forward(self._model, data)
                err = target - np.asarray(data.xpos[self._ee], dtype=np.float64)
                if float(np.linalg.norm(err)) < 1e-4:
                    break
                mujoco.mj_jacBody(self._model, data, jacp, None, self._ee)
                j = jacp[:, :7]
                a = j @ j.T + DAMPING * np.eye(3)
                dq = j.T @ np.linalg.solve(a, err)
                dq = np.clip(dq, -STEP_CLIP, STEP_CLIP)
                q = np.clip(q + dq, self._lo, self._hi)
            self._q = q
        except np.linalg.LinAlgError:
            pass
        finally:
            data.qpos[:7] = q_save
            mujoco.mj_forward(self._model, data)
        return self._q.copy()
