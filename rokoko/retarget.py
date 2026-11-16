"""Rokoko Smartgloves → Orca hand joint targets.

Rokoko Studio's custom stream (JSON v3) names the 21 right-hand joints in the same order
MediaPipe uses (wrist, then thumb/index/middle/ring/little as proximal, medial, distal, tip),
so the glove frame goes straight into the existing ``orca_teleop`` retargeter as a
``source="mediapipe"`` (21, 3) array. The only conversion is handedness: Rokoko positions are
in a left-handed Y-up/Z-forward frame, and the retargeter's canonical hand frame assumes a
right-handed one (checked on rokoko/samples: curled fingertips landed on the back-of-hand side
until z was negated). Translation/rotation are removed by the retargeter itself and its
auto-scale calibration absorbs the hand size (positions are already metres), so **hold the hand
open and flat for the first second** after start/reset, like with the webcam.

The orca_teleop config ships with ``lp_alpha: 1.0`` (no smoothing), and on glove data the
solver occasionally snaps a joint limit-to-limit in one frame, so this wrapper adds an EMA
(``smoothing``), a per-frame step limit (``max_step_deg``) and a clip to the sim joint ranges.

    retargeter = RokokoHandRetargeter(model_path, urdf_path)
    q = retargeter(frame)          # frame: Rokoko JSON dict, or a (21, 3) positions array
    # q: OrcaJointPositions (degrees) or None during the calibration window

``joint_array(q)`` gives the plain float array in ``joint_ids`` order for the sim.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from orca_core import OrcaJointPositions
from orca_teleop.retargeting.retargeter import Retargeter, TargetPose
from orca_teleop.retargeting.utils import get_normalized_local_manohand_joint_pos

from rokoko.stream import frame_to_positions

# Rokoko (left-handed) → right-handed: negate z.
HANDEDNESS_FLIP = np.array([1.0, 1.0, -1.0])

# models/nero_orca_scene.xml joint ranges (deg); the solver's own bounds are wider for the thumb.
SIM_JOINT_RANGE_DEG: dict[str, tuple[float, float]] = {
    "wrist": (-65, 35),
    "thumb_cmc": (-45, 33), "thumb_abd": (-18, 55), "thumb_mcp": (-25, 100), "thumb_dip": (-15, 107),
    "index_abd": (-30, 25), "index_mcp": (-25, 100), "index_pip": (-15, 107),
    "middle_abd": (-27, 27), "middle_mcp": (-25, 100), "middle_pip": (-15, 107),
    "ring_abd": (-27, 27), "ring_mcp": (-25, 100), "ring_pip": (-15, 107),
    "pinky_abd": (-30, 30), "pinky_mcp": (-25, 100), "pinky_pip": (-15, 107),
}


# Finger abduction straight from the glove geometry. The retargeter's abd output barely moves
# on glove data (ring 10 -> 5 deg between a full spread and fingers together, and every finger
# leans to the pinky side), because Rokoko's finger-spread signal is small: fingertip gaps change
# only ~2 cm. So each finger's proximal phalanx angle in the palm plane (deg, + toward the index
# side; robust during a fist, unlike the full-finger direction) is mapped linearly from the
# measured spread/together angles (rokoko/samples/abd_test.npz, one operator) to Orca abd targets
# (+ = pinky side) that visibly spread / touch in the sim.
ABD_CALIB: dict[str, tuple[tuple[float, float], tuple[float, float]]] = {
    # finger: ((angle_spread, abd_spread), (angle_together, abd_together))
    "index": ((10.4, -25.0), (-5.0, 20.0)),
    "middle": ((0.1, -10.0), (-6.5, 8.0)),
    "ring": ((-13.7, 15.0), (-9.3, -8.0)),
    "pinky": ((-24.3, 28.0), (-13.1, -25.0)),
}
_FINGER_BASE = {"index": 5, "middle": 9, "ring": 13, "pinky": 17}
# Fade the geometric abd toward a loose "together" value as the finger curls (mcp 30 -> 70 deg),
# since the proximal phalanx swings sideways in a fist and the real pose there is fingers together.
ABD_FADE_MCP_DEG = (30.0, 70.0)
ABD_FIST_FRACTION = 0.6


# Thumb straight from the glove geometry as well: the IK retargeter has several thumb solutions
# and settles in a different one after each curl (visible as the thumb "moving once" after
# unfolding), and its thumb_dip sat at the lower limit most of the time. The glove exposes
# ~3 usable thumb DoF: MCP flex (angle between proximal and middle phalanx), IP flex, and the
# proximal phalanx direction in the palm frame (in-plane angle, + toward the index side;
# elevation out of the palm plane, + palm side). Each is a piecewise-linear map, calibrated on
# rokoko/samples/thumb_test.npz (relaxed 0-7 s, curl/opposition 8-10 and 17-19 s, spread 11-15
# and 21-23 s) to Orca thumb joints: cmc (+ across the palm toward the pinky, - outward),
# abd (+ lifts toward the index / palm plane), mcp and dip (flex).
THUMB_CALIB: dict[str, list[tuple[float, float]]] = {
    # joint: [(glove_value, orca_deg), ...] sorted by glove_value; interpolated, clamped at ends
    "thumb_mcp": [(15.0, 0.0), (73.0, 90.0)],          # glove MCP flex deg
    "thumb_dip": [(3.0, 0.0), (48.0, 90.0)],           # glove IP flex deg
    "thumb_cmc": [(-25.0, 33.0), (12.0, 0.0), (25.0, -45.0)],   # in-plane angle deg
    "thumb_abd": [(27.0, -10.0), (36.0, 25.0), (40.0, 45.0)],   # elevation deg
}


def _interp(table: list[tuple[float, float]], x: float) -> float:
    xs, ys = zip(*table, strict=True)
    return float(np.interp(x, xs, ys))


def thumb_from_canonical(canonical: np.ndarray) -> dict[str, float]:
    """Orca thumb joints (deg) from canonical (21, 3) hand points; see ``THUMB_CALIB``."""
    cmc, mcp, ip, tip = canonical[1], canonical[2], canonical[3], canonical[4]

    def angle(a: np.ndarray, b: np.ndarray) -> float:
        c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9))
        return math.degrees(math.acos(max(-1.0, min(1.0, c))))

    prox = mcp - cmc
    return {
        "thumb_mcp": _interp(THUMB_CALIB["thumb_mcp"], angle(prox, ip - mcp)),
        "thumb_dip": _interp(THUMB_CALIB["thumb_dip"], angle(ip - mcp, tip - ip)),
        "thumb_cmc": _interp(THUMB_CALIB["thumb_cmc"], math.degrees(math.atan2(prox[0], prox[1]))),
        "thumb_abd": _interp(
            THUMB_CALIB["thumb_abd"], math.degrees(math.atan2(prox[2], math.hypot(prox[0], prox[1])))
        ),
    }


def abduction_from_canonical(canonical: np.ndarray) -> dict[str, float]:
    """Per-finger Orca abd (deg, + = pinky side) from canonical (21, 3) hand points."""
    out = {}
    for finger, base in _FINGER_BASE.items():
        v = canonical[base + 1] - canonical[base]
        angle = math.degrees(math.atan2(v[0], v[1]))
        (a0, q0), (a1, q1) = ABD_CALIB[finger]
        out[finger] = q0 + (angle - a0) * (q1 - q0) / (a1 - a0)
    return out


class RokokoHandRetargeter:
    """Rokoko right-hand keypoints → Orca joint targets via the orca_teleop retargeter.

    ``smoothing`` is the EMA weight of the new sample (1.0 = off); ``max_step_deg`` caps the
    change per call (at 30 Hz, 12 deg/frame = 360 deg/s); ``joint_range`` clips the output
    (defaults to the sim ranges, ``None`` disables); ``geometric_abd`` replaces the four finger
    abd outputs with ``abduction_from_canonical`` (see ``ABD_CALIB``) and ``geometric_thumb``
    the four thumb joints with ``thumb_from_canonical`` (see ``THUMB_CALIB``).
    """

    def __init__(
        self,
        model_path: str | None,
        urdf_path: str | None,
        *,
        backend: str = "adaptive_analytical",
        config_path: str | None = None,
        smoothing: float = 0.5,
        max_step_deg: float | None = 12.0,
        joint_range: dict[str, tuple[float, float]] | None = SIM_JOINT_RANGE_DEG,
        geometric_abd: bool = True,
        geometric_thumb: bool = True,
    ) -> None:
        self._retargeter = Retargeter.from_paths(
            model_path, urdf_path, backend=backend, config_path=config_path
        )
        self.joint_ids: list[str] = list(self._retargeter._hand.config.joint_ids)  # noqa: SLF001
        if not 0.0 < smoothing <= 1.0:
            raise ValueError(f"smoothing must be in (0, 1], got {smoothing}")
        self._alpha = float(smoothing)
        self._max_step = None if max_step_deg is None else float(max_step_deg)
        self._geometric_abd = bool(geometric_abd)
        self._geometric_thumb = bool(geometric_thumb)
        if joint_range:
            self._lo = np.array([joint_range.get(j, (-np.inf, np.inf))[0] for j in self.joint_ids])
            self._hi = np.array([joint_range.get(j, (-np.inf, np.inf))[1] for j in self.joint_ids])
        else:
            self._lo = self._hi = None
        self._state: np.ndarray | None = None

    def reset(self) -> None:
        """Restart the auto-scale calibration (hold the hand open) and the filters."""
        self._retargeter.reset()
        self._state = None

    def __call__(self, frame: dict[str, Any] | np.ndarray) -> OrcaJointPositions | None:
        if isinstance(frame, dict):
            positions = frame_to_positions(frame)
            if positions is None:
                return None
        else:
            positions = np.asarray(frame, dtype=float)
        if positions.shape != (21, 3):
            raise ValueError(f"expected (21, 3) positions, got {positions.shape}")
        positions = positions * HANDEDNESS_FLIP
        pose = TargetPose(joint_positions=positions, source="mediapipe")
        raw = self._retargeter.retarget(pose)
        if raw is None:
            return None
        q = self._as_array(raw)
        if self._geometric_abd or self._geometric_thumb:
            self._override_geometric(q, positions)
        q = self._postprocess(q)
        return OrcaJointPositions(dict(zip(self.joint_ids, q.tolist(), strict=True)))

    def _override_geometric(self, q: np.ndarray, positions: np.ndarray) -> None:
        canonical = get_normalized_local_manohand_joint_pos(positions, "mediapipe")
        if self._geometric_thumb:
            for joint, deg in thumb_from_canonical(canonical).items():
                q[self.joint_ids.index(joint)] = deg
        if not self._geometric_abd:
            return
        lo_mcp, hi_mcp = ABD_FADE_MCP_DEG
        for finger, abd in abduction_from_canonical(canonical).items():
            j_abd = self.joint_ids.index(f"{finger}_abd")
            mcp = q[self.joint_ids.index(f"{finger}_mcp")]
            w = float(np.clip(1.0 - (mcp - lo_mcp) / (hi_mcp - lo_mcp), 0.0, 1.0))
            fist = ABD_CALIB[finger][1][1] * ABD_FIST_FRACTION
            q[j_abd] = w * abd + (1.0 - w) * fist

    def joint_array(self, q: OrcaJointPositions) -> np.ndarray:
        """Joint targets (degrees) as a float array in ``joint_ids`` order."""
        return self._as_array(q)

    def _as_array(self, q: OrcaJointPositions) -> np.ndarray:
        d = q.to_dict() if hasattr(q, "to_dict") else dict(q)
        return np.array([float(d.get(j, 0.0)) for j in self.joint_ids])

    def _postprocess(self, q: np.ndarray) -> np.ndarray:
        if self._lo is not None:
            q = np.clip(q, self._lo, self._hi)
        if self._state is None:
            self._state = q.copy()
            return q
        target = self._state + self._alpha * (q - self._state)
        if self._max_step is not None:
            target = self._state + np.clip(target - self._state, -self._max_step, self._max_step)
        self._state = target
        return target.copy()
