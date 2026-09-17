#!/usr/bin/env python3
"""Recordable sink: Nero wrist-position IK + Orca finger teleop, in sim or on hardware.

The webcam retargeter still emits ``OrcaJointPositions`` for the fingers. MediaPipe
also streams the wrist position (image x, y, palm width [+ RGB-D X, Y, Z]); this
sink maps that delta to a carpals target around the rest EE and solves damped
least-squares IK for Nero. Quest/Manus frames have no wrist position, so the arm
stays at rest. Dataset vector:

    concat([nero_1..7, orca wrist + 16 fingers])

Wrist is forced to 0 in commands (MediaPipe does not drive it). Units in the
dataset are degrees, matching the existing orca_teleop LeRobot schema.

Each device has an output mode (see ``hardware.py``): ``sim`` runs MuJoCo physics,
``shadow`` reads the real joints and shows the command as a translucent ghost,
``real`` also sends the command. With any hardware device the MuJoCo state mirrors
real feedback, so the viewer and the recorded frontal render show the real robot.
The viewer runs on its own thread with its own ``MjData`` and never blocks control.

    python combined_sink.py --check
"""

from __future__ import annotations

import argparse
import logging
import math
import queue
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
import orca_core
from orca_core import OrcaHandConfig, OrcaJointPositions

from arm_ik import NeroArmIK, WristPosition, WristPositionMirror
from hardware import (
    NERO_DEFAULT_SPEED_PERCENT,
    NERO_MAX_JOINT_SPEED,
    FakeNero,
    FakeOrca,
    NeroDevice,
    NeroOutput,
    OrcaCoreHand,
    OrcaDevice,
    OrcaOutput,
    PyAgxNero,
)
from limits import REST_QPOS, apply_rest_pose, nero_ctrl, step_nero
from orca_teleop.cameras import CameraManager, OpenCVCameraConfig
from orca_teleop.pipeline import _SHUTDOWN, RecordableSink, SinkObservation
from orca_teleop.sim import SimCameraConfig, _sim_joint_to_config_id
from orca_teleop.utils import RateTicker

logger = logging.getLogger(__name__)

ROOT = Path(__file__).resolve().parent
SCENE_XML = ROOT / "models" / "nero_orca_scene.xml"

NERO_JOINT_IDS = [f"nero_joint{i}" for i in range(1, 8)]
HAND_PREFIX = "orca_right_"
MAX_SUBSTEPS = 50
RECORD_WIDTH = 320
RECORD_HEIGHT = 240
VIEWER_HZ = 15.0
IDLE_REFRESH_S = 0.15  # hardware mirror refresh when no teleop command arrives
GHOST_RGBA = np.array([0.2, 0.8, 1.0, 0.25], dtype=np.float32)
HOME_HAND_STEPS = 50


def _orca_core_config_path(hand: str = "right") -> Path:
    name = f"orcahand-{hand}"
    candidates = [
        Path(orca_core.__file__).resolve().parent / "models" / "v2" / name / "config.yaml",
        ROOT.parent / "orca_core" / "orca_core" / "models" / "v2" / name / "config.yaml",
    ]
    for path in candidates:
        if path.exists():
            return path
    raise FileNotFoundError(
        "missing orca_core hand config; looked in:\n  " + "\n  ".join(str(p) for p in candidates)
    )


def _actuator_core_name(mj_joint_name: str) -> str:
    name = mj_joint_name
    if name.startswith(HAND_PREFIX):
        name = name[len(HAND_PREFIX) :]
    if name.endswith("_actuator"):
        name = name[: -len("_actuator")]
    return name


class CombinedNeroOrcaSink(RecordableSink):
    """Drive ``nero_orca_scene.xml`` and optionally the real Nero arm / Orca hand."""

    def __init__(
        self,
        scene_xml: Path | None = None,
        camera_config: SimCameraConfig | None = None,
        camera_configs: list[OpenCVCameraConfig] | None = None,
        control_hz: float = 15.0,
        show_viewer: bool = True,
        nero_output: str = "sim",
        orca_output: str = "sim",
        nero_can: str = "can0",
        nero_speed_percent: int = NERO_DEFAULT_SPEED_PERCENT,
        orca_model_path: str | None = None,
        nero_device: NeroDevice | None = None,
        orca_device: OrcaDevice | None = None,
    ) -> None:
        self._scene_xml = Path(scene_xml) if scene_xml is not None else SCENE_XML
        self._camera_config = camera_config or SimCameraConfig(
            name="frontal", width=RECORD_WIDTH, height=RECORD_HEIGHT, shadows=False
        )
        self._cameras = CameraManager(camera_configs or [])
        self._control_hz = float(control_hz)
        self._show_viewer = bool(show_viewer)
        if nero_output != "sim" and nero_device is None:
            nero_device = PyAgxNero(channel=nero_can)
        if orca_output != "sim" and orca_device is None:
            # Default to the same v2 config the combined MJCF is built from (orca_core's
            # own default may be v1, whose joint ids do not match the model).
            orca_device = OrcaCoreHand(orca_model_path or str(_orca_core_config_path("right")))
        self._nero = NeroOutput(nero_output, nero_device, nero_speed_percent)
        self._orca = OrcaOutput(orca_output, orca_device)
        self._lock = threading.RLock()
        self._model: Any = None
        self._data: Any = None
        self._renderer: Any = None
        self._record_camera: Any = None
        self._viewer: Any = None
        self._viewer_data: Any = None
        self._ghost_data: Any = None
        self._viewer_thread: threading.Thread | None = None
        self._viewer_stop = threading.Event()
        self._hand_config: Any = None
        self._retarget_model_path: str | None = None
        self._orca_joint_ids: list[str] = []
        self._orca_qpos_adr: dict[str, int] = {}
        self._orca_act_id: dict[str, int] = {}
        self._nero_act_id: list[int] = []
        self._nero_qpos_adr: list[int] = []
        self._nero_dof_adr: list[int] = []
        self._dataset_joint_ids: list[str] = []
        self._last_hand: OrcaJointPositions | None = None
        self._last_step_time: float | None = None
        self._timestep = 0.002
        self._wrist_positions = WristPositionMirror()
        self._arm_ik: NeroArmIK | None = None
        self._nero_cmd = np.array(REST_QPOS, dtype=np.float64)

    @property
    def uses_hardware(self) -> bool:
        return self._nero.hardware or self._orca.hardware

    @property
    def output_modes(self) -> dict[str, str]:
        return {"nero": self._nero.mode, "orca": self._orca.mode}

    def connect(self) -> None:
        import mujoco

        if not self._scene_xml.exists():
            raise FileNotFoundError(f"run attach_orca.py first: missing {self._scene_xml}")

        model = mujoco.MjModel.from_xml_path(str(self._scene_xml))
        data = mujoco.MjData(model)
        self._model = model
        self._data = data
        self._timestep = float(model.opt.timestep)

        if self._orca.hardware:
            # The physical hand's own config (port, calibration) also drives retargeting.
            self._hand_config = self._orca.config
            self._retarget_model_path = str(self._hand_config.config_path)
        else:
            cfg_path = _orca_core_config_path("right")
            self._retarget_model_path = str(cfg_path)
            self._hand_config = OrcaHandConfig.from_config_path(config_path=str(cfg_path))
        OrcaJointPositions.register_joint_names(self._hand_config.joint_ids)
        self._map_actuators(mujoco, model)

        apply_rest_pose(model, data)
        self._wrist_positions.reset()
        self._arm_ik = NeroArmIK(model)
        self._arm_ik.capture_rest_ee(data)
        self._nero_cmd = np.array(REST_QPOS, dtype=np.float64)
        self._last_hand = self.home_position()

        self._nero.connect()
        self._orca.connect()
        self._nero.move_home()
        self._settle(0.5)

        self._renderer = mujoco.Renderer(
            model,
            height=self._camera_config.height,
            width=self._camera_config.width,
        )
        if not self._camera_config.shadows:
            flags = self._renderer.scene.flags
            flags[int(mujoco.mjtRndFlag.mjRND_SHADOW)] = 0
            flags[int(mujoco.mjtRndFlag.mjRND_REFLECTION)] = 0
        self._record_camera = mujoco.MjvCamera()
        mujoco.mjv_defaultFreeCamera(model, self._record_camera)
        self._record_camera.lookat[:] = [0.0, 0.0, 0.45]
        self._record_camera.distance = 1.6
        self._record_camera.azimuth = 140.0
        self._record_camera.elevation = -20.0

        if self._camera_config.name in self._cameras.names:
            raise ValueError(
                f"Real camera name {self._camera_config.name!r} collides with the sim "
                "render camera name; rename it so both observations get distinct keys."
            )
        self._cameras.open()
        self._cameras.ensure_live()
        self._last_step_time = time.perf_counter()
        self._open_viewer(mujoco, model)
        logger.info(
            "Combined sink connected: xml=%s dataset_dofs=%d dt=%.4fs cameras=%s "
            "viewer=%s nero=%s orca=%s",
            self._scene_xml,
            len(self._dataset_joint_ids),
            self._timestep,
            self.camera_shapes,
            self._viewer is not None,
            self._nero.mode,
            self._orca.mode,
        )

    # ------------------------------------------------------------------ viewer

    def _open_viewer(self, mujoco: Any, model: Any) -> None:
        if not self._show_viewer:
            return
        try:
            from mujoco import viewer

            self._viewer_data = mujoco.MjData(model)
            self._ghost_data = mujoco.MjData(model)
            self._copy_view_state()
            vis = viewer.launch_passive(model, self._viewer_data)
            vis.user_scn.flags[int(mujoco.mjtRndFlag.mjRND_SHADOW)] = 0
            vis.user_scn.flags[int(mujoco.mjtRndFlag.mjRND_REFLECTION)] = 0
            vis.sync()
            self._viewer = vis
            logger.info(
                "MuJoCo 창 제목: Nero_orca_view  (%s)",
                "실물 상태 + 반투명 명령 고스트" if self.uses_hardware else "시뮬 상태",
            )
        except Exception:
            logger.exception("MuJoCo viewer failed to open; continuing without it")
            self._viewer = None
            return
        self._viewer_stop.clear()
        self._viewer_thread = threading.Thread(
            target=self._viewer_loop, name="mujoco-viewer", daemon=True
        )
        self._viewer_thread.start()

    def _copy_view_state(self) -> None:
        """Snapshot state (and command ghost) into the viewer-owned MjData copies."""
        assert self._data is not None
        with self._lock:
            self._viewer_data.qpos[:] = self._data.qpos
            self._ghost_data.qpos[:] = self._data.qpos
            for adr, q in zip(self._nero_qpos_adr, self._nero_cmd):
                self._ghost_data.qpos[adr] = q
            hand = self._last_hand.data if self._last_hand is not None else {}
            for jid, adr in self._orca_qpos_adr.items():
                deg = 0.0 if jid == "wrist" else float(hand.get(jid) or 0.0)
                self._ghost_data.qpos[adr] = math.radians(deg)

    def _viewer_loop(self) -> None:
        import mujoco

        opt = mujoco.MjvOption()
        pert = mujoco.MjvPerturb()
        catmask = int(mujoco.mjtCatBit.mjCAT_DYNAMIC)
        period = 1.0 / VIEWER_HZ
        while not self._viewer_stop.is_set():
            vis = self._viewer
            if vis is None:
                return
            t0 = time.perf_counter()
            try:
                if not vis.is_running():
                    self._viewer = None
                    return
                if self.uses_hardware and self._last_step_time is not None:
                    if t0 - self._last_step_time > IDLE_REFRESH_S:
                        self.refresh()
                self._copy_view_state()
                with vis.lock():
                    mujoco.mj_forward(self._model, self._viewer_data)
                    scn = vis.user_scn
                    scn.ngeom = 0
                    if self.uses_hardware:
                        mujoco.mj_forward(self._model, self._ghost_data)
                        mujoco.mjv_addGeoms(
                            self._model, self._ghost_data, opt, pert, catmask, scn
                        )
                        for i in range(scn.ngeom):
                            scn.geoms[i].rgba[:] = GHOST_RGBA
                vis.sync()
            except Exception:
                logger.exception("MuJoCo viewer update failed; closing the viewer")
                self._viewer = None
                return
            self._viewer_stop.wait(max(0.0, period - (time.perf_counter() - t0)))

    @property
    def viewer_running(self) -> bool:
        return self._viewer is not None

    # ------------------------------------------------------------------ mapping

    def _map_actuators(self, mujoco: Any, model: Any) -> None:
        valid = set(self._hand_config.joint_ids)
        orca_ids: list[str] = []
        unmapped: list[str] = []
        for i in range(model.nu):
            name = model.actuator(i).name
            if name.startswith("nero_"):
                continue
            jnt_id = int(model.actuator_trnid[i, 0])
            mj_joint = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jnt_id) or ""
            config_id = _sim_joint_to_config_id(_actuator_core_name(mj_joint), valid)
            if config_id is None:
                unmapped.append(name)
                continue
            orca_ids.append(config_id)
            self._orca_act_id[config_id] = i
            self._orca_qpos_adr[config_id] = int(model.jnt_qposadr[jnt_id])

        if unmapped:
            raise ValueError(f"combined-model Orca actuators have no orca_core id: {unmapped}")
        if set(orca_ids) != valid or len(set(orca_ids)) != len(orca_ids):
            raise ValueError(
                f"Orca actuator map is not 1:1 with orca_core. "
                f"Mapped {sorted(orca_ids)} vs config {sorted(valid)}"
            )
        # Dataset order: Nero 7, then orca_core joint_ids (wrist first).
        self._orca_joint_ids = list(self._hand_config.joint_ids)
        self._nero_act_id = []
        self._nero_qpos_adr = []
        self._nero_dof_adr = []
        for spec_name in NERO_JOINT_IDS:
            act_name = f"{spec_name}_act"
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, spec_name.replace("nero_", ""))
            if aid < 0 or jid < 0:
                raise RuntimeError(f"missing Nero actuator/joint for {spec_name}")
            self._nero_act_id.append(aid)
            self._nero_qpos_adr.append(int(model.jnt_qposadr[jid]))
            self._nero_dof_adr.append(int(model.jnt_dofadr[jid]))
        self._dataset_joint_ids = NERO_JOINT_IDS + self._orca_joint_ids

    # ------------------------------------------------------------------ sink API

    @property
    def joint_ids(self) -> list[str]:
        return list(self._dataset_joint_ids)

    @property
    def retarget_model_path(self) -> str | None:
        return self._retarget_model_path

    @property
    def camera_shapes(self) -> dict[str, tuple[int, int, int]]:
        return {
            self._camera_config.name: (
                self._camera_config.height,
                self._camera_config.width,
                3,
            ),
            **self._cameras.shapes,
        }

    def home_position(self) -> OrcaJointPositions:
        assert self._hand_config is not None
        positions = dict(self._hand_config.neutral_position)
        positions["wrist"] = 0.0
        return OrcaJointPositions(positions)

    def update_wrist_position(self, landmarks: object) -> None:
        self._wrist_positions.update_from_landmarks(landmarks)

    def go_home(self) -> None:
        assert self._model is not None and self._data is not None
        with self._lock:
            self._wrist_positions.reset()
            self._last_hand = self.home_position()
            self._nero_cmd = np.array(REST_QPOS, dtype=np.float64)
            if not self._nero.hardware:
                apply_rest_pose(self._model, self._data)
        # Blocking hardware moves stay outside the lock so the viewer keeps mirroring.
        self._orca.send(self._hand_command(self._last_hand), num_steps=HOME_HAND_STEPS)
        self._nero.move_home()
        with self._lock:
            if self._arm_ik is not None:
                q_save = np.array(self._data.qpos, dtype=np.float64)
                self._arm_ik.capture_rest_ee(self._data)
                self._data.qpos[:] = q_save
            self._settle(0.5)

    def dataset_action(self, hand_action: OrcaJointPositions) -> np.ndarray:
        """24-DoF command for LeRobot: Nero IK cmd (deg) + Orca command (deg)."""
        nero = np.rad2deg(np.asarray(self._nero_cmd, dtype=np.float64))
        orca = np.empty(len(self._orca_joint_ids), dtype=np.float64)
        for i, jid in enumerate(self._orca_joint_ids):
            if jid == "wrist":
                orca[i] = 0.0
            else:
                orca[i] = float(hand_action.data.get(jid, 0.0))
        return np.concatenate([nero, orca]).astype(np.float32)

    def get_observation(self) -> SinkObservation:
        with self._lock:
            return SinkObservation(
                joint_state=self._read_joint_state(),
                images={self._camera_config.name: self._render(), **self._cameras.capture()},
            )

    def joint_state(self) -> np.ndarray:
        """Dataset-ordered joint state (deg) without rendering."""
        with self._lock:
            return self._read_joint_state()

    def _read_joint_state(self) -> np.ndarray:
        assert self._data is not None
        nero = np.rad2deg(
            np.array([float(self._data.qpos[adr]) for adr in self._nero_qpos_adr], dtype=np.float64)
        )
        orca = np.rad2deg(
            np.array(
                [float(self._data.qpos[self._orca_qpos_adr[jid]]) for jid in self._orca_joint_ids],
                dtype=np.float64,
            )
        )
        return np.concatenate([nero, orca]).astype(np.float32)

    def _render(self) -> np.ndarray:
        assert self._renderer is not None and self._record_camera is not None
        self._renderer.update_scene(self._data, camera=self._record_camera)
        return np.asarray(self._renderer.render(), dtype=np.uint8)

    # ------------------------------------------------------------------ control

    def _hand_command(self, action: OrcaJointPositions) -> OrcaJointPositions:
        positions = dict(action.data)
        positions["wrist"] = 0.0
        return OrcaJointPositions(positions)

    def _write_hand_ctrl(self, positions_deg: dict[str, float], force_wrist_zero: bool) -> None:
        assert self._data is not None
        for jid, aid in self._orca_act_id.items():
            if jid == "wrist" and force_wrist_zero:
                self._data.ctrl[aid] = 0.0
                continue
            deg = positions_deg.get(jid)
            self._data.ctrl[aid] = math.radians(float(deg or 0.0))

    def dispatch_action(self, action: OrcaJointPositions) -> None:
        assert self._model is not None and self._data is not None
        with self._lock:
            self._last_hand = action
            prev_cmd = self._nero_cmd.copy()
            if self._arm_ik is not None:
                self._nero_cmd = self._arm_ik.solve(self._data, self._wrist_positions.snapshot())
            self._nero.send(self._nero_cmd)
            self._orca.send(self._hand_command(action))
            self._nero.refresh()
            self._orca.refresh()
            self._advance(prev_cmd)

    def refresh(self) -> None:
        """Re-read hardware feedback and update the MuJoCo mirror (no commands sent)."""
        if self._data is None:
            return
        with self._lock:
            self._nero.refresh()
            self._orca.refresh()
            self._advance(self._nero_cmd.copy())

    def _settle(self, seconds: float) -> None:
        with self._lock:
            self._last_step_time = None
            for _ in range(max(1, int(round(seconds * self._control_hz)))):
                nstep = max(1, int(1.0 / (self._control_hz * self._timestep)))
                self._advance(self._nero_cmd.copy(), nstep=nstep)

    def _advance(self, prev_cmd: np.ndarray, nstep: int | None = None) -> None:
        """Step physics toward the command (sim) or pin hardware joints to feedback."""
        import mujoco

        now = time.perf_counter()
        if nstep is None:
            if self._last_step_time is None:
                nstep = max(1, int(round(1.0 / (self._control_hz * self._timestep))))
            else:
                nstep = int(round((now - self._last_step_time) / self._timestep))
                nstep = int(np.clip(nstep, 1, MAX_SUBSTEPS))
        self._last_step_time = now

        nero_q = self._nero.q
        hand = self._orca.deg if self._orca.hardware else (self._last_hand or self.home_position()).data
        self._write_hand_ctrl(hand or {}, force_wrist_zero=not self._orca.hardware)
        for k in range(nstep):
            if self._nero.hardware:
                target = nero_q if nero_q is not None else prev_cmd
            else:
                # Ramp the arm command across substeps instead of a 15 Hz staircase.
                alpha = (k + 1) / nstep
                target = prev_cmd + alpha * (self._nero_cmd - prev_cmd)
            step_nero(self._model, self._data, nero_ctrl(target))
            if self._nero.hardware and nero_q is not None:
                for adr, dof, q in zip(self._nero_qpos_adr, self._nero_dof_adr, nero_q):
                    self._data.qpos[adr] = q
                    self._data.qvel[dof] = 0.0
        if self._nero.hardware:
            mujoco.mj_forward(self._model, self._data)

    def run_loop(
        self,
        actions_q: queue.Queue[OrcaJointPositions | object],
        stop_event: threading.Event,
    ) -> None:
        assert self._last_hand is not None
        ticker = RateTicker(dt=1.0 / self._control_hz)
        latest = self._last_hand
        while not stop_event.is_set():
            shutdown = False
            try:
                item = actions_q.get_nowait()
                if item is _SHUTDOWN:
                    shutdown = True
                elif isinstance(item, OrcaJointPositions):
                    latest = item
            except queue.Empty:
                pass
            if shutdown:
                break
            try:
                self.dispatch_action(latest)
            except Exception:
                logger.exception("combined sink step failed")
                break
            ticker.tick()

    def close(self) -> None:
        if self._model is None:
            return
        self._viewer_stop.set()
        if self._viewer_thread is not None:
            self._viewer_thread.join(timeout=2.0)
        try:
            self._cameras.close()
            if self._renderer is not None:
                self._renderer.close()
            vis = self._viewer
            if vis is not None:
                try:
                    vis.close()
                except Exception:
                    pass
        except Exception:
            logger.exception("Combined sink close failed")
        finally:
            self._nero.close()
            self._orca.close()
            self._model = None
            self._data = None
            self._renderer = None
            self._record_camera = None
            self._viewer = None
            self._viewer_thread = None


def _fake_landmarks(wrist_position: np.ndarray) -> object:
    return type("L", (), {"wrist_position": wrist_position})()


def check_sink() -> None:
    sink = CombinedNeroOrcaSink(show_viewer=False)
    sink.connect()
    ids = sink.joint_ids
    print("dataset joint_ids", ids)
    if len(ids) != 24:
        raise SystemExit(f"expected 24 dataset dofs, got {len(ids)}")
    if ids[:7] != NERO_JOINT_IDS:
        raise SystemExit(f"Nero names should lead the schema: {ids[:7]}")
    if ids[7] != "wrist":
        raise SystemExit(f"orca_core wrist should follow Nero, got {ids[7]!r}")

    home = sink.home_position()
    sink.dispatch_action(home)
    obs = sink.get_observation()
    if obs.joint_state.shape != (24,):
        raise SystemExit(f"state shape {obs.joint_state.shape}, expected (24,)")
    act = sink.dataset_action(home)
    if act.shape != (24,):
        raise SystemExit(f"action shape {act.shape}, expected (24,)")
    if abs(float(act[3]) - math.degrees(1.22)) > 0.05:
        raise SystemExit(f"Nero joint4 command should be rest 1.22 rad, got {act[3]}")
    if abs(float(act[7])) > 1e-6:
        raise SystemExit(f"wrist command should be 0, got {act[7]}")

    flexed = dict(home.as_dict())
    flexed["index_mcp"] = math.degrees(0.8)
    sink.dispatch_action(OrcaJointPositions(flexed))
    for _ in range(8):
        sink.dispatch_action(OrcaJointPositions(flexed))
    after = sink.get_observation().joint_state
    idx = ids.index("index_mcp")
    print("hold nero deg", [round(float(v), 2) for v in after[:7]])
    print(f"index_mcp deg={after[idx]:.2f}  target={flexed['index_mcp']:.2f}")
    if abs(float(after[idx]) - flexed["index_mcp"]) > 12.0:
        raise SystemExit("index_mcp did not track the named command")
    rest_err = max(abs(float(after[i]) - math.degrees(REST_QPOS[i])) for i in range(7))
    if rest_err > 12.0:
        raise SystemExit(f"arm drifted off rest pose without a wrist position ({rest_err:.2f} deg)")

    origin = WristPosition(wrist_image=np.array([0.50, 0.50, 0.12], dtype=np.float64))
    moved = WristPosition(wrist_image=np.array([0.78, 0.28, 0.20], dtype=np.float64))
    sink._wrist_positions.update_from_landmarks(_fake_landmarks(origin.wrist_image))
    sink.dispatch_action(OrcaJointPositions(flexed))
    for _ in range(25):
        sink._wrist_positions.update_from_landmarks(_fake_landmarks(moved.wrist_image))
        time.sleep(0.04)
        sink.dispatch_action(OrcaJointPositions(flexed))
    after_ik = sink.get_observation().joint_state
    act_ik = sink.dataset_action(OrcaJointPositions(flexed))
    arm_state = max(abs(float(after_ik[i]) - math.degrees(REST_QPOS[i])) for i in range(7))
    arm_cmd = max(abs(float(act_ik[i]) - math.degrees(REST_QPOS[i])) for i in range(7))
    print("ik nero deg", [round(float(v), 2) for v in after_ik[:7]])
    print(f"index_mcp after ik deg={after_ik[idx]:.2f}")
    print(f"arm left rest: state={arm_state:.2f} deg  cmd={arm_cmd:.2f} deg")
    if arm_cmd < 8.0:
        raise SystemExit("wrist-image IK did not change the Nero command")
    if arm_state < 5.0:
        raise SystemExit("wrist-image IK did not move the arm off rest")
    if abs(float(after_ik[idx]) - flexed["index_mcp"]) > 12.0:
        raise SystemExit("index_mcp stopped tracking after IK")

    # RGB-D wrist position: palm 10 cm right and 10 cm farther → EE should move ~+Y and ~+X.
    sink.go_home()
    ee_id = sink._arm_ik._ee
    ee_rest = np.array(sink._data.xpos[ee_id], dtype=np.float64)
    rgbd_origin = np.array([0.50, 0.50, 0.12, 0.00, 0.00, 0.60], dtype=np.float64)
    rgbd_moved = np.array([0.65, 0.50, 0.10, 0.10, 0.00, 0.70], dtype=np.float64)
    sink._wrist_positions.update_from_landmarks(_fake_landmarks(rgbd_origin))
    sink.dispatch_action(OrcaJointPositions(flexed))
    for _ in range(40):
        sink._wrist_positions.update_from_landmarks(_fake_landmarks(rgbd_moved))
        time.sleep(0.04)
        sink.dispatch_action(OrcaJointPositions(flexed))
    ee_move = np.array(sink._data.xpos[ee_id], dtype=np.float64) - ee_rest
    print("rgbd ee move (m)", np.round(ee_move, 3))
    if ee_move[0] < 0.05 or ee_move[1] < 0.05:
        raise SystemExit("RGB-D metric wrist position did not move the EE toward +X/+Y")
    img = obs.images["frontal"]
    if img.ndim != 3 or img.shape[2] != 3:
        raise SystemExit(f"bad render shape {img.shape}")
    sink.close()
    print("ok: combined sink tracks fingers and moves Nero from wrist-position IK")


def check_hardware_modes() -> None:
    """Shadow/real output paths against in-memory fake devices (no hardware needed)."""
    cfg = OrcaHandConfig.from_config_path(config_path=str(_orca_core_config_path("right")))
    rng = np.random.default_rng(0)
    q_real = np.array(REST_QPOS) + rng.uniform(-0.2, 0.2, 7)
    q_real[3] = 1.0  # stay inside joint4 limits

    # Shadow: mirror real feedback, never send.
    nero = FakeNero(q_real)
    orca_deg = {jid: float(v) for jid, v in cfg.neutral_position.items()}
    orca_deg["index_mcp"] = 30.0
    orca = FakeOrca(cfg, orca_deg)
    sink = CombinedNeroOrcaSink(
        show_viewer=False, nero_output="shadow", orca_output="shadow",
        nero_device=nero, orca_device=orca,
    )
    sink.connect()
    home = sink.home_position()
    flexed = dict(home.as_dict())
    flexed["index_mcp"] = 70.0
    sink._wrist_positions.update_from_landmarks(
        _fake_landmarks(np.array([0.5, 0.5, 0.12, 0.0, 0.0, 0.6]))
    )
    for _ in range(10):
        sink._wrist_positions.update_from_landmarks(
            _fake_landmarks(np.array([0.6, 0.5, 0.12, 0.1, 0.0, 0.7]))
        )
        sink.dispatch_action(OrcaJointPositions(flexed))
        time.sleep(0.02)
    state = sink.joint_state()
    ids = sink.joint_ids
    arm_err = float(np.max(np.abs(state[:7] - np.degrees(q_real))))
    print(f"shadow: arm state vs real feedback max err {arm_err:.3f} deg, sent={len(nero.sent)}")
    if nero.sent or orca.sent or nero.enabled or orca.enabled:
        raise SystemExit("shadow mode enabled or commanded hardware")
    if arm_err > 0.05:
        raise SystemExit("shadow mode arm state does not mirror real feedback")
    if float(np.max(np.abs(sink._nero_cmd - np.array(REST_QPOS)))) < 0.01:
        raise SystemExit("shadow mode did not compute an IK command")
    sink.close()

    # Real: enable, send rate-limited commands, force wrist 0 on the hand.
    nero = FakeNero(np.array(REST_QPOS))
    orca = FakeOrca(cfg, {jid: float(v) for jid, v in cfg.neutral_position.items()})
    sink = CombinedNeroOrcaSink(
        show_viewer=False, nero_output="real", orca_output="real",
        nero_device=nero, orca_device=orca,
    )
    sink.connect()
    if not (nero.enabled and orca.enabled):
        raise SystemExit("real mode did not enable devices")
    sink._wrist_positions.update_from_landmarks(
        _fake_landmarks(np.array([0.5, 0.5, 0.12, 0.0, 0.0, 0.6]))
    )
    n0 = len(nero.sent)
    last = time.perf_counter()
    max_step_ratio = 0.0
    for _ in range(30):
        sink._wrist_positions.update_from_landmarks(
            _fake_landmarks(np.array([0.6, 0.5, 0.12, 0.15, 0.0, 0.75]))
        )
        before = nero.sent[-1].copy() if nero.sent else None
        sink.dispatch_action(OrcaJointPositions(flexed))
        now = time.perf_counter()
        if before is not None and len(nero.sent) > n0:
            step = float(np.max(np.abs(nero.sent[-1] - before)))
            max_step_ratio = max(max_step_ratio, step / (NERO_MAX_JOINT_SPEED * min(now - last, 0.2) + 1e-6))
        last = now
        time.sleep(0.03)
    print(
        f"real: nero commands={len(nero.sent) - n0}, max step / speed cap={max_step_ratio:.2f}, "
        f"hand commands={len(orca.sent)}"
    )
    if len(nero.sent) - n0 < 10:
        raise SystemExit("real mode did not stream Nero commands")
    if max_step_ratio > 1.3:
        raise SystemExit("real mode Nero command exceeded the joint speed cap")
    if not orca.sent or orca.sent[-1].get("wrist") != 0.0:
        raise SystemExit("real mode hand command missing or wrist not forced to 0")
    if abs(orca.sent[-1].get("index_mcp", 0.0) - 70.0) > 1e-6:
        raise SystemExit("real mode hand command did not carry the finger target")
    idx = ids.index("index_mcp")
    if abs(float(sink.joint_state()[idx]) - 70.0) > 12.0:
        raise SystemExit("real mode hand state does not mirror hand feedback")
    sink.close()
    print("ok: shadow mirrors without sending; real sends rate-limited commands")


def main() -> None:
    parser = argparse.ArgumentParser(description="Combined Nero+Orca sink")
    parser.add_argument("--check", action="store_true", help="sim + fake-hardware self-checks")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    if args.check:
        check_sink()
        check_hardware_modes()
        return
    parser.error("pass --check, or use record_dataset.py / mirror_real.py")


if __name__ == "__main__":
    main()
