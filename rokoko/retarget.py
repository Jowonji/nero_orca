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

Finger abduction and the thumb come from the glove geometry with per-operator tables
(``HandCalibration``): ``calibrate=True`` runs ``GuidedCalibrator`` (three prompted poses after
the scale window) and saves to ``calib_file`` (default rokoko/calib/hand.json), which later
instances load automatically; without a file the built-in constants are used.
"""

from __future__ import annotations

import json
import logging
import math
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
from orca_core import OrcaJointPositions
from orca_teleop.retargeting.retargeter import Retargeter, TargetPose
from orca_teleop.retargeting.utils import get_normalized_local_manohand_joint_pos

from rokoko.stream import frame_to_positions

logger = logging.getLogger(__name__)

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


# Orca targets the calibrated glove extremes map to (deg); fixed across operators, only the
# glove-side values in ABD_CALIB / THUMB_CALIB change with the hand wearing the glove.
ABD_TARGET_DEG: dict[str, tuple[float, float]] = {  # finger: (spread, together)
    finger: (spread[1], together[1]) for finger, (spread, together) in ABD_CALIB.items()
}
THUMB_TARGET_DEG: dict[str, tuple[float, ...]] = {  # joint: outputs in THUMB_CALIB order
    joint: tuple(y for _, y in table) for joint, table in THUMB_CALIB.items()
}
# Minimum glove-side separation (deg) between two calibration poses for the measurement to be
# trusted; below it the default table for that joint is kept.
MIN_CALIB_SEPARATION_DEG = 2.5
# Thumb flexion spans ~60 deg open→fist; a smaller spread means the thumb was not curled in the
# fist pose (seen 11/17: dip 4→12 would have mapped 8 deg of glove to 90 deg of Orca).
MIN_THUMB_FLEX_SEPARATION_DEG = 25.0

# Finger flexion (mcp/pip from the IK retargeter) is also rescaled per operator: the open pose
# maps to 0 and the fist to these targets. The retargeter's range differs per finger on glove
# data (index mcp tops out ~70 deg while the middle reaches ~85, so the real index never closed).
FLEX_JOINTS = tuple(f"{f}_{j}" for f in ("index", "middle", "ring", "pinky") for j in ("mcp", "pip"))
FLEX_TARGET_DEG = {"mcp": 90.0, "pip": 100.0}
MIN_FLEX_SEPARATION_DEG = 20.0

# Where RokokoHandRetargeter looks for the operator calibration by default (written by the
# guided calibration, see GuidedCalibrator); absent → built-in constants.
DEFAULT_CALIB_FILE = Path(__file__).resolve().parent / "calib" / "hand.json"


def _interp(table: list[tuple[float, float]], x: float) -> float:
    xs, ys = zip(*table, strict=True)
    return float(np.interp(x, xs, ys))


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    c = float(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def raw_signals(canonical: np.ndarray) -> dict[str, float]:
    """Glove-side signals (deg) the geometric maps are calibrated on.

    ``index``..``pinky``: proximal phalanx angle in the palm plane (+ toward the index side).
    ``thumb_mcp``/``thumb_dip``: flexion between phalanges. ``thumb_cmc``: proximal phalanx
    in-plane angle (+ toward the index side). ``thumb_abd``: its elevation (+ palm side).
    """
    out: dict[str, float] = {}
    for finger, base in _FINGER_BASE.items():
        v = canonical[base + 1] - canonical[base]
        out[finger] = math.degrees(math.atan2(v[0], v[1]))
    cmc, mcp, ip, tip = canonical[1], canonical[2], canonical[3], canonical[4]
    prox = mcp - cmc
    out["thumb_mcp"] = _angle(prox, ip - mcp)
    out["thumb_dip"] = _angle(ip - mcp, tip - ip)
    out["thumb_cmc"] = math.degrees(math.atan2(prox[0], prox[1]))
    out["thumb_abd"] = math.degrees(math.atan2(prox[2], math.hypot(prox[0], prox[1])))
    return out


@dataclass
class HandCalibration:
    """Glove→Orca tables for one operator (same layout as ``ABD_CALIB`` / ``THUMB_CALIB``)."""

    abd: dict[str, tuple[tuple[float, float], tuple[float, float]]]
    thumb: dict[str, list[tuple[float, float]]]
    # mcp/pip joint: ((retargeter_open, 0), (retargeter_fist, target)); missing = pass-through
    flex: dict[str, tuple[tuple[float, float], tuple[float, float]]] = field(default_factory=dict)
    source: str = "built-in constants"

    def save(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "abd": {f: [list(a), list(b)] for f, (a, b) in self.abd.items()},
            "thumb": {j: [list(p) for p in table] for j, table in self.thumb.items()},
            "flex": {j: [list(a), list(b)] for j, (a, b) in self.flex.items()},
            "source": self.source,
        }
        path.write_text(json.dumps(payload, indent=2) + "\n")

    @classmethod
    def load(cls, path: str | Path) -> "HandCalibration":
        d = json.loads(Path(path).read_text())
        return cls(
            abd={f: (tuple(a), tuple(b)) for f, (a, b) in d["abd"].items()},
            thumb={j: [tuple(p) for p in table] for j, table in d["thumb"].items()},
            flex={j: (tuple(a), tuple(b)) for j, (a, b) in d.get("flex", {}).items()},
            source=d.get("source", str(path)),
        )

    def apply_flex(self, joint: str, value: float) -> float:
        """Rescale one retargeter mcp/pip output (deg) with the operator's open/fist range."""
        table = self.flex.get(joint)
        if table is None:
            return value
        (x0, y0), (x1, y1) = table
        return y0 + (value - x0) * (y1 - y0) / (x1 - x0)

    @classmethod
    def from_poses(
        cls,
        together: dict[str, float],
        spread: dict[str, float],
        fist: dict[str, float],
    ) -> tuple["HandCalibration", list[str]]:
        """Build tables from the median ``raw_signals`` of the three guided poses.

        Fixed Orca targets (``ABD_TARGET_DEG`` / ``THUMB_TARGET_DEG``) are attached to the
        measured glove values. A joint whose poses are not separated/ordered as expected keeps
        its default table; those are reported in the returned warnings. Pose dicts may also
        carry the retargeter's flexion outputs as ``q:<joint>`` (see ``GuidedCalibrator``),
        which give the per-finger ``flex`` rescaling.
        """
        warnings: list[str] = []
        gap = MIN_CALIB_SEPARATION_DEG
        flex: dict[str, tuple[tuple[float, float], tuple[float, float]]] = {}
        for joint in FLEX_JOINTS:
            key = f"q:{joint}"
            if key not in together or key not in fist:
                continue
            q_open, q_fist = together[key], fist[key]
            if q_fist - q_open < MIN_FLEX_SEPARATION_DEG:
                warnings.append(f"{joint}: open/fist too close ({q_open:.0f}/{q_fist:.0f})")
                continue
            flex[joint] = ((q_open, 0.0), (q_fist, FLEX_TARGET_DEG[joint.rsplit("_", 1)[1]]))

        abd = dict(ABD_CALIB)
        for finger, (t_spread, t_together) in ABD_TARGET_DEG.items():
            a_sp, a_tg = spread[finger], together[finger]
            if abs(a_sp - a_tg) < gap:
                warnings.append(f"{finger}_abd: spread/together too close ({a_sp:.1f}/{a_tg:.1f})")
                continue
            abd[finger] = ((a_sp, t_spread), (a_tg, t_together))

        thumb = dict(THUMB_CALIB)
        # (glove values in table order, joint) — tables need strictly increasing glove values.
        candidates = {
            "thumb_mcp": (together["thumb_mcp"], fist["thumb_mcp"]),
            "thumb_dip": (together["thumb_dip"], fist["thumb_dip"]),
            "thumb_cmc": (fist["thumb_cmc"], together["thumb_cmc"], spread["thumb_cmc"]),
            "thumb_abd": (spread["thumb_abd"], together["thumb_abd"], fist["thumb_abd"]),
        }
        for joint, xs in candidates.items():
            need = MIN_THUMB_FLEX_SEPARATION_DEG if joint in ("thumb_mcp", "thumb_dip") else gap
            if all(b - a >= need for a, b in zip(xs[:-1], xs[1:], strict=True)):
                thumb[joint] = list(zip(xs, THUMB_TARGET_DEG[joint], strict=True))
            else:
                warnings.append(f"{joint}: poses not separated as expected {tuple(round(x, 1) for x in xs)}")
        return cls(abd=abd, thumb=thumb, flex=flex, source="guided calibration"), warnings

    def describe(self) -> str:
        abd = ", ".join(f"{f} {a[0]:.0f}/{b[0]:.0f}" for f, (a, b) in self.abd.items())
        thumb = ", ".join(f"{j} {'/'.join(f'{x:.0f}' for x, _ in t)}" for j, t in self.thumb.items())
        flex = ", ".join(f"{j} {a[0]:.0f}/{b[0]:.0f}" for j, (a, b) in self.flex.items()) or "pass-through"
        return f"{self.source}: abd spread/together [{abd}]; thumb [{thumb}]; flex open/fist [{flex}]"


DEFAULT_CALIBRATION = HandCalibration(abd=ABD_CALIB, thumb=THUMB_CALIB)


def thumb_from_canonical(
    canonical: np.ndarray, calib: HandCalibration = DEFAULT_CALIBRATION
) -> dict[str, float]:
    """Orca thumb joints (deg) from canonical (21, 3) hand points; see ``THUMB_CALIB``."""
    sig = raw_signals(canonical)
    return {joint: _interp(calib.thumb[joint], sig[joint]) for joint in THUMB_CALIB}


def hand_pitch(positions: np.ndarray) -> float | None:
    """Wrist→middle-MCP tilt below horizontal (rad, down positive) from raw Rokoko positions.

    Same definition as ``arm_ik._hand_pitch`` for MediaPipe, but Rokoko's world frame is Y up.
    """
    d = np.asarray(positions, dtype=float)[9] - np.asarray(positions, dtype=float)[0]
    if not np.all(np.isfinite(d)) or float(np.linalg.norm(d)) < 1e-6:
        return None
    return float(math.atan2(-d[1], math.hypot(d[0], d[2])))


def abduction_from_canonical(
    canonical: np.ndarray, calib: HandCalibration = DEFAULT_CALIBRATION
) -> dict[str, float]:
    """Per-finger Orca abd (deg, + = pinky side) from canonical (21, 3) hand points."""
    sig = raw_signals(canonical)
    out = {}
    for finger in _FINGER_BASE:
        (a0, q0), (a1, q1) = calib.abd[finger]
        out[finger] = q0 + (sig[finger] - a0) * (q1 - q0) / (a1 - a0)
    return out


# Guided calibration timing: seconds to get into each pose, then seconds of samples taken.
CALIB_MOVE_S = 4.0
CALIB_SAMPLE_S = 2.0

# Guided calibration poses, in order: (key, instruction shown to the operator).
CALIB_POSES: tuple[tuple[str, str], ...] = (
    ("together", "손을 펴고 손가락을 전부 붙이기, 엄지는 편하게 (open hand, fingers together)"),
    ("spread", "손가락과 엄지를 최대한 벌리기 (spread all fingers and the thumb)"),
    ("fist", "주먹 쥐기, 엄지도 접기 (fist, thumb curled)"),
)


class GuidedCalibrator:
    """Three timed poses → ``HandCalibration``; feed it canonical frames via ``update``.

    Each pose gets ``move_s`` to get into position, then ``sample_s`` of samples whose medians
    go to ``HandCalibration.from_poses``. ``say`` receives the operator prompts.
    """

    def __init__(
        self,
        move_s: float = CALIB_MOVE_S,
        sample_s: float = CALIB_SAMPLE_S,
        say: Callable[[str], None] | None = None,
    ) -> None:
        self._move_s = float(move_s)
        self._sample_s = float(sample_s)
        self._say = say or logger.info
        self._index = -1
        self._phase_start = 0.0
        self._countdown = 0
        self._samples: list[dict[str, float]] = []
        self._poses: dict[str, dict[str, float]] = {}
        self.result: HandCalibration | None = None
        self.warnings: list[str] = []

    @property
    def done(self) -> bool:
        return self.result is not None

    def update(
        self, canonical: np.ndarray, t: float, flex: dict[str, float] | None = None
    ) -> HandCalibration | None:
        """Advance with one canonical (21, 3) frame at time ``t`` (s); returns the result once.

        ``flex`` are the retargeter's current mcp/pip outputs (deg) for the flexion rescaling.
        """
        if self.done:
            return None
        if self._index < 0:
            self._start_pose(0, t)
        elapsed = t - self._phase_start
        remaining = math.ceil(self._move_s - elapsed)
        if 0 < remaining < self._countdown:
            self._countdown = remaining
            self._say(f"[calib {self._index + 1}/{len(CALIB_POSES)}]   {remaining}...")
        if elapsed >= self._move_s:
            if not self._samples:
                self._say(f"[calib {self._index + 1}/{len(CALIB_POSES)}] 측정 중 — 그대로 유지 ({self._sample_s:.0f}s)")
            sample = raw_signals(canonical)
            if flex:
                sample.update({f"q:{j}": float(v) for j, v in flex.items()})
            self._samples.append(sample)
        if elapsed >= self._move_s + self._sample_s:
            key = CALIB_POSES[self._index][0]
            keys = self._samples[0].keys()
            self._poses[key] = {k: float(np.median([s[k] for s in self._samples])) for k in keys}
            if self._index + 1 < len(CALIB_POSES):
                self._start_pose(self._index + 1, t)
                return None
            self.result, self.warnings = HandCalibration.from_poses(
                self._poses["together"], self._poses["spread"], self._poses["fist"]
            )
            for w in self.warnings:
                self._say(f"[calib] 기본값 유지: {w}")
            self._say(f"[calib] 완료 — {self.result.describe()}")
            return self.result
        return None

    def _start_pose(self, index: int, t: float) -> None:
        self._index = index
        self._phase_start = t
        self._samples = []
        self._countdown = math.ceil(self._move_s)
        _, text = CALIB_POSES[index]
        self._say(f"[calib {index + 1}/{len(CALIB_POSES)}] {text} — {self._move_s:.0f}s 후 측정")


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
        calib_file: str | Path | None = DEFAULT_CALIB_FILE,
        calibrate: bool = False,
        say: Callable[[str], None] | None = None,
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

        self._say = say or logger.info
        self._calib_file = Path(calib_file) if calib_file else None
        self.calibration = DEFAULT_CALIBRATION
        if not calibrate and self._calib_file and self._calib_file.exists():
            self.calibration = HandCalibration.load(self._calib_file)
            self.calibration.source = f"file {self._calib_file}"
        self._say(f"glove calibration: {self.calibration.describe()}")
        self._calibrator: GuidedCalibrator | None = None
        self._calibrate_on_start = bool(calibrate)
        if calibrate:
            self._calibrator = GuidedCalibrator(say=self._say)

    @property
    def calibrating(self) -> bool:
        """True while the guided pose sequence is still collecting."""
        return self._calibrator is not None

    def reset(self) -> None:
        """Restart the auto-scale calibration (hold the hand open) and the filters."""
        self._retargeter.reset()
        self._state = None
        if self._calibrate_on_start:
            self._calibrator = GuidedCalibrator(say=self._say)

    def __call__(
        self, frame: dict[str, Any] | np.ndarray, t: float | None = None
    ) -> OrcaJointPositions | None:
        """``t`` (s) only matters during guided calibration; defaults to the wall clock."""
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
        if self._geometric_abd or self._geometric_thumb or self._calibrator is not None:
            canonical = get_normalized_local_manohand_joint_pos(positions, "mediapipe")
            if self._calibrator is not None:
                # Runs after the retargeter's own scale window, so the hand is already tracked.
                flex = {j: q[self.joint_ids.index(j)] for j in FLEX_JOINTS if j in self.joint_ids}
                result = self._calibrator.update(
                    canonical, time.monotonic() if t is None else t, flex
                )
                if result is not None:
                    self.calibration = result
                    self._calibrator = None
                    if self._calib_file:
                        result.save(self._calib_file)
                        self._say(f"[calib] saved to {self._calib_file}")
            self._apply_flex(q)
            self._override_geometric(q, canonical)
        q = self._postprocess(q)
        return OrcaJointPositions(dict(zip(self.joint_ids, q.tolist(), strict=True)))

    def _apply_flex(self, q: np.ndarray) -> None:
        if not self.calibration.flex:
            return
        for joint in FLEX_JOINTS:
            if joint in self.calibration.flex:
                i = self.joint_ids.index(joint)
                q[i] = self.calibration.apply_flex(joint, q[i])

    def _override_geometric(self, q: np.ndarray, canonical: np.ndarray) -> None:
        if self._geometric_thumb:
            for joint, deg in thumb_from_canonical(canonical, self.calibration).items():
                q[self.joint_ids.index(joint)] = deg
        if not self._geometric_abd:
            return
        lo_mcp, hi_mcp = ABD_FADE_MCP_DEG
        for finger, abd in abduction_from_canonical(canonical, self.calibration).items():
            j_abd = self.joint_ids.index(f"{finger}_abd")
            mcp = q[self.joint_ids.index(f"{finger}_mcp")]
            w = float(np.clip(1.0 - (mcp - lo_mcp) / (hi_mcp - lo_mcp), 0.0, 1.0))
            fist = ABD_TARGET_DEG[finger][1] * ABD_FIST_FRACTION
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
