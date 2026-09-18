"""Real Nero arm (pyAgxArm over CAN) and Orca hand (orca_core over USB) drivers.

Each device runs in one of three output modes:

- ``sim``: no hardware; MuJoCo physics follows the command.
- ``shadow``: connect and read joint state only. Commands are computed and drawn
  as a ghost in the viewer but never sent (no enable, no torque).
- ``real``: connect, enable, and send commands with conservative safety limits.

Joint angles are radians for Nero and degrees (``OrcaJointPositions``) for Orca,
matching each SDK. ``NERO_JOINT_SIGN`` / ``NERO_JOINT_OFFSET`` map real → model
angles; verify them with ``mirror_real.py`` before using ``real``.
"""

from __future__ import annotations

import logging
import time
from typing import Protocol

import numpy as np

from limits import JOINTS, REST_QPOS

logger = logging.getLogger(__name__)

OUTPUT_MODES = ("sim", "shadow", "real")

# model_q = sign * real_q + offset (rad). Identity until checked on the real arm.
NERO_JOINT_SIGN = np.ones(7, dtype=np.float64)
NERO_JOINT_OFFSET = np.zeros(7, dtype=np.float64)

NERO_DEFAULT_SPEED_PERCENT = 20
NERO_MAX_JOINT_SPEED = 0.6  # rad/s cap on streamed teleop commands
NERO_MAX_TRACKING_ERROR = 0.35  # rad; larger command-vs-state gap → hold, don't send
NERO_FEEDBACK_TIMEOUT = 0.25  # s; older joint feedback → hold, don't send
NERO_CONNECT_TIMEOUT = 3.0
NERO_HOME_TIMEOUT = 20.0
NERO_HOME_TOLERANCE = 0.03  # rad


class NeroDevice(Protocol):
    def connect(self) -> None: ...
    def read_q(self) -> tuple[np.ndarray, float] | None: ...
    def enable(self, speed_percent: int) -> None: ...
    def send_q(self, q: np.ndarray) -> None: ...
    def close(self) -> None: ...


class OrcaDevice(Protocol):
    config: object

    def connect(self) -> None: ...
    def read_deg(self) -> dict[str, float]: ...
    def enable(self) -> None: ...
    def send_deg(self, positions: object, num_steps: int = 1) -> None: ...
    def close(self) -> None: ...


def model_from_real(q_real: np.ndarray) -> np.ndarray:
    return NERO_JOINT_SIGN * np.asarray(q_real, dtype=np.float64) + NERO_JOINT_OFFSET


def real_from_model(q_model: np.ndarray) -> np.ndarray:
    return (np.asarray(q_model, dtype=np.float64) - NERO_JOINT_OFFSET) * NERO_JOINT_SIGN


class PyAgxNero:
    """Nero over pyAgxArm. Angles in and out are model-frame radians."""

    def __init__(self, channel: str = "can0", interface: str = "socketcan") -> None:
        self._channel = channel
        self._interface = interface
        self._robot = None
        self._feedback_ts: float | None = None
        self._feedback_seen = 0.0

    def connect(self) -> None:
        from pyAgxArm import AgxArmFactory, ArmModel, NeroFW, create_agx_arm_config

        cfg = create_agx_arm_config(
            robot=ArmModel.NERO,
            firmeware_version=NeroFW.DEFAULT,
            interface=self._interface,
            channel=self._channel,
        )
        self._robot = AgxArmFactory.create_arm(cfg)
        self._robot.connect()
        deadline = time.monotonic() + NERO_CONNECT_TIMEOUT
        while self.read_q() is None:
            if time.monotonic() > deadline:
                raise RuntimeError(
                    f"no Nero joint feedback on {self._interface}:{self._channel} within "
                    f"{NERO_CONNECT_TIMEOUT:.0f}s (arm powered? CAN up at 1000000 bit/s?)"
                )
            time.sleep(0.05)
        logger.info("Nero connected on %s:%s", self._interface, self._channel)

    def read_q(self) -> tuple[np.ndarray, float] | None:
        assert self._robot is not None
        ja = self._robot.get_joint_angles()
        if ja is None:
            return None
        # The SDK returns its last cached frame; only a new CAN timestamp counts as fresh.
        if ja.timestamp != self._feedback_ts:
            self._feedback_ts = ja.timestamp
            self._feedback_seen = time.monotonic()
        return model_from_real(np.asarray(ja.msg, dtype=np.float64)), self._feedback_seen

    def enable(self, speed_percent: int) -> None:
        assert self._robot is not None
        deadline = time.monotonic() + NERO_CONNECT_TIMEOUT
        while not self._robot.enable():
            if time.monotonic() > deadline:
                raise RuntimeError("Nero joints did not enable")
            time.sleep(0.01)
        self._robot.set_speed_percent(int(speed_percent))
        self._robot.set_motion_mode(self._robot.OPTIONS.MOTION_MODE.J)
        logger.warning("Nero ENABLED (speed %d%%)", speed_percent)

    def send_q(self, q: np.ndarray) -> None:
        assert self._robot is not None
        self._robot.move_j([float(v) for v in real_from_model(q)])

    def close(self) -> None:
        # Joints stay enabled: disabling an arm carrying the hand can let it drop.
        if self._robot is not None:
            logger.warning("Nero left enabled at its last pose; disable it from a safe pose")
        self._robot = None


class OrcaCoreHand:
    """Orca hand over orca_core. Positions are degrees keyed by joint id."""

    def __init__(self, model_path: str | None = None) -> None:
        """``model_path``: orca_core hand ``config.yaml`` (or model dir, orca_core permitting)."""
        from orca_core import OrcaHand

        self._hand = OrcaHand(model_path)
        self.config = self._hand.config
        self._enabled = False

    def connect(self) -> None:
        ok, message = self._hand.connect()
        if not ok:
            raise RuntimeError(f"Orca hand failed to connect: {message}")
        logger.info("Orca hand connected (%s)", message)
        if not self._hand.calibrated:
            logger.warning(
                "Orca hand is NOT calibrated (%s): joint readings are unreliable, and "
                "real mode will run calibration first (motors sweep to their limits)",
                self.config.calibration_path,
            )

    def read_deg(self) -> dict[str, float]:
        return dict(self._hand.get_joint_position().as_dict())

    def enable(self) -> None:
        self._hand.init_joints()  # torque on, calibrate if needed, move to neutral
        self._enabled = True
        logger.warning("Orca hand torque ENABLED")

    def send_deg(self, positions: object, num_steps: int = 1) -> None:
        self._hand.set_joint_positions(positions, num_steps=num_steps)

    def close(self) -> None:
        try:
            if self._enabled:
                self._hand.set_zero_position()
                self._hand.disable_torque()
            self._hand.disconnect()
        except Exception:
            logger.exception("Orca hand close failed")


class NeroOutput:
    """Mode-aware Nero link: feedback cache plus rate/tracking-limited sending."""

    def __init__(self, mode: str, device: NeroDevice | None, speed_percent: int) -> None:
        if mode not in OUTPUT_MODES:
            raise ValueError(f"nero output must be one of {OUTPUT_MODES}, got {mode!r}")
        if mode != "sim" and device is None:
            raise ValueError(f"nero output {mode!r} needs a device")
        self.mode = mode
        self._device = device
        self._speed_percent = int(speed_percent)
        self._lo = np.array([spec["sim_lower"] for spec in JOINTS], dtype=np.float64)
        self._hi = np.array([spec["sim_upper"] for spec in JOINTS], dtype=np.float64)
        self._q: np.ndarray | None = None
        self._stamp = 0.0
        self._last_sent: np.ndarray | None = None
        self._last_send_time: float | None = None
        self._hold_reason = ""

    @property
    def hardware(self) -> bool:
        return self.mode != "sim"

    def connect(self) -> None:
        if not self.hardware:
            return
        assert self._device is not None
        self._device.connect()
        self.refresh()
        if self.mode == "real":
            self._device.enable(self._speed_percent)

    def refresh(self) -> np.ndarray | None:
        if not self.hardware:
            return None
        assert self._device is not None
        sample = self._device.read_q()
        if sample is not None:
            self._q, self._stamp = sample
        return self._q

    @property
    def q(self) -> np.ndarray | None:
        return None if self._q is None else self._q.copy()

    def feedback_fresh(self) -> bool:
        return self._q is not None and time.monotonic() - self._stamp <= NERO_FEEDBACK_TIMEOUT

    def move_home(self, q_home: np.ndarray | None = None) -> None:
        """Blocking joint move to rest (real mode only)."""
        if self.mode != "real":
            return
        assert self._device is not None
        target = np.clip(np.asarray(REST_QPOS if q_home is None else q_home), self._lo, self._hi)
        logger.warning("Nero moving to rest pose at %d%% speed", self._speed_percent)
        self._device.send_q(target)
        deadline = time.monotonic() + NERO_HOME_TIMEOUT
        while time.monotonic() < deadline:
            q = self.refresh()
            if q is not None and float(np.max(np.abs(q - target))) < NERO_HOME_TOLERANCE:
                break
            time.sleep(0.05)
        else:
            logger.error("Nero did not reach rest within %.0fs", NERO_HOME_TIMEOUT)
        self._last_sent = target.copy()
        self._last_send_time = time.monotonic()

    def send(self, q_cmd: np.ndarray) -> np.ndarray | None:
        """Send a clipped, rate-limited command in real mode. Returns what was sent."""
        if self.mode != "real":
            return None
        assert self._device is not None
        q = self.refresh()
        if not self.feedback_fresh() or q is None:
            return self._hold("stale Nero joint feedback")
        target = np.clip(np.asarray(q_cmd, dtype=np.float64), self._lo, self._hi)
        now = time.monotonic()
        base = self._last_sent if self._last_sent is not None else q
        dt = 0.1 if self._last_send_time is None else min(now - self._last_send_time, 0.2)
        step = np.clip(target - base, -NERO_MAX_JOINT_SPEED * dt, NERO_MAX_JOINT_SPEED * dt)
        target = base + step
        err = float(np.max(np.abs(target - q)))
        if err > NERO_MAX_TRACKING_ERROR:
            # base may be stale (e.g. an unreached move_home target); re-anchor on
            # the arm's actual position so the next call steps from where it
            # really is instead of holding forever against a gap that can
            # never close (base was frozen and never advances while held).
            self._last_sent = None
            return self._hold(f"Nero tracking error {np.degrees(err):.1f} deg")
        self._device.send_q(target)
        self._last_sent = target
        self._last_send_time = now
        self._hold_reason = ""
        return target

    def _hold(self, reason: str) -> None:
        if reason != self._hold_reason:
            logger.warning("Nero command held: %s", reason)
            self._hold_reason = reason
        return None

    def close(self) -> None:
        if self._device is not None and self.hardware:
            self._device.close()


class OrcaOutput:
    """Mode-aware Orca link: feedback cache plus sending in real mode."""

    def __init__(self, mode: str, device: OrcaDevice | None) -> None:
        if mode not in OUTPUT_MODES:
            raise ValueError(f"orca output must be one of {OUTPUT_MODES}, got {mode!r}")
        if mode != "sim" and device is None:
            raise ValueError(f"orca output {mode!r} needs a device")
        self.mode = mode
        self._device = device
        self._deg: dict[str, float] | None = None

    @property
    def hardware(self) -> bool:
        return self.mode != "sim"

    @property
    def config(self) -> object | None:
        return None if self._device is None else self._device.config

    def connect(self) -> None:
        if not self.hardware:
            return
        assert self._device is not None
        self._device.connect()
        if self.mode == "real":
            self._device.enable()
        self.refresh()

    def refresh(self) -> dict[str, float] | None:
        if not self.hardware:
            return None
        assert self._device is not None
        self._deg = self._device.read_deg()
        return self._deg

    @property
    def deg(self) -> dict[str, float] | None:
        return None if self._deg is None else dict(self._deg)

    def send(self, positions: object, num_steps: int = 1) -> None:
        if self.mode == "real":
            assert self._device is not None
            self._device.send_deg(positions, num_steps=num_steps)

    def close(self) -> None:
        if self._device is not None and self.hardware:
            self._device.close()


class FakeNero:
    """In-memory Nero for checks: state moves to the last command at a fixed speed."""

    def __init__(self, q0: np.ndarray | None = None) -> None:
        self.q = np.array(REST_QPOS if q0 is None else q0, dtype=np.float64)
        self.enabled = False
        self.sent: list[np.ndarray] = []

    def connect(self) -> None:
        pass

    def read_q(self) -> tuple[np.ndarray, float] | None:
        if self.sent:
            self.q = self.q + np.clip(self.sent[-1] - self.q, -0.05, 0.05)
        return self.q.copy(), time.monotonic()

    def enable(self, speed_percent: int) -> None:
        self.enabled = True

    def send_q(self, q: np.ndarray) -> None:
        if not self.enabled:
            raise RuntimeError("FakeNero: command sent while disabled")
        self.sent.append(np.asarray(q, dtype=np.float64).copy())

    def close(self) -> None:
        pass


class FakeOrca:
    """In-memory Orca hand for checks."""

    def __init__(self, config: object, deg: dict[str, float]) -> None:
        self.config = config
        self.deg = dict(deg)
        self.enabled = False
        self.sent: list[dict[str, float]] = []

    def connect(self) -> None:
        pass

    def read_deg(self) -> dict[str, float]:
        return dict(self.deg)

    def enable(self) -> None:
        self.enabled = True

    def send_deg(self, positions: object, num_steps: int = 1) -> None:
        if not self.enabled:
            raise RuntimeError("FakeOrca: command sent while torque is off")
        data = dict(getattr(positions, "data", positions))
        self.sent.append(data)
        self.deg.update({k: float(v) for k, v in data.items() if v is not None})

    def close(self) -> None:
        pass
