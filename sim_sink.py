#!/usr/bin/env python3
"""Recordable sink: Nero wrist-image IK + Orca finger teleop on the combined MJCF.

The webcam retargeter still emits ``OrcaJointPositions`` for the fingers. MediaPipe
also streams image-space wrist (x, y) + palm width; this sink maps that delta to
a carpals target around the rest EE and solves damped least-squares IK for Nero.
Quest/Manus frames have no wrist image, so the arm stays at rest. Dataset vector:

    concat([nero_1..7, orca wrist + 16 fingers])

Wrist is forced to 0 (MediaPipe does not drive it). Units in the dataset are
degrees, matching the existing orca_teleop LeRobot schema.

    conda activate orca   # or orca_teleop .venv
    python sim_sink.py --check
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

from arm_ik import ArmHint, ArmHintMirror, NeroArmIK
from limits import REST_QPOS, apply_rest_pose, nero_ctrl, step_nero
from orca_teleop.cameras import CameraManager, OpenCVCameraConfig
from orca_teleop.pipeline import RecordableSink, SinkObservation, _SHUTDOWN
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


class CombinedNeroOrcaSimSink(RecordableSink):
    """Drive ``nero_orca_scene.xml``: Nero IK from wrist image + named Orca fingers."""

    def __init__(
        self,
        scene_xml: Path | None = None,
        camera_config: SimCameraConfig | None = None,
        camera_configs: list[OpenCVCameraConfig] | None = None,
        control_hz: float = 15.0,
        show_viewer: bool = True,
    ) -> None:
        self._scene_xml = Path(scene_xml) if scene_xml is not None else SCENE_XML
        self._camera_config = camera_config or SimCameraConfig(
            name="frontal", width=RECORD_WIDTH, height=RECORD_HEIGHT, shadows=False
        )
        self._cameras = CameraManager(camera_configs or [])
        self._control_hz = float(control_hz)
        self._show_viewer = bool(show_viewer)
        self._lock = threading.Lock()
        self._model: Any = None
        self._data: Any = None
        self._renderer: Any = None
        self._record_camera: Any = None
        self._viewer: Any = None
        self._next_viewer_sync = 0.0
        self._viewer_hz = 15.0
        self._hand_config: Any = None
        self._retarget_model_path: str | None = None
        self._orca_joint_ids: list[str] = []
        self._orca_qpos_adr: dict[str, int] = {}
        self._orca_act_id: dict[str, int] = {}
        self._nero_act_id: list[int] = []
        self._nero_qpos_adr: list[int] = []
        self._dataset_joint_ids: list[str] = []
        self._last_hand: OrcaJointPositions | None = None
        self._last_step_time: float | None = None
        self._timestep = 0.002
        self._arm_hints = ArmHintMirror()
        self._arm_ik: NeroArmIK | None = None
        self._nero_cmd = np.array(REST_QPOS, dtype=np.float64)

    def connect(self) -> None:
        import mujoco

        if not self._scene_xml.exists():
            raise FileNotFoundError(f"run attach_orca.py first: missing {self._scene_xml}")

        model = mujoco.MjModel.from_xml_path(str(self._scene_xml))
        data = mujoco.MjData(model)
        self._model = model
        self._data = data
        self._timestep = float(model.opt.timestep)

        cfg_path = _orca_core_config_path("right")
        self._retarget_model_path = str(cfg_path)
        self._hand_config = OrcaHandConfig.from_config_path(config_path=str(cfg_path))
        OrcaJointPositions.register_joint_names(self._hand_config.joint_ids)
        self._map_actuators(mujoco, model)

        apply_rest_pose(model, data)
        self._arm_hints.reset()
        self._arm_ik = NeroArmIK(model)
        self._arm_ik.capture_rest_ee(data)
        self._nero_cmd = np.array(REST_QPOS, dtype=np.float64)
        self._last_hand = self.home_position()
        self._write_hand_ctrl(self._last_hand)
        for _ in range(int(round(0.5 / self._timestep))):
            step_nero(model, data, nero_ctrl(self._nero_cmd))

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
        self._open_viewer(mujoco, model, data)
        logger.info(
            "Combined SimSink connected: xml=%s dataset_dofs=%d dt=%.4fs cameras=%s viewer=%s",
            self._scene_xml,
            len(self._dataset_joint_ids),
            self._timestep,
            self.camera_shapes,
            self._viewer is not None,
        )

    def _open_viewer(self, mujoco: Any, model: Any, data: Any) -> None:
        if not self._show_viewer:
            return
        try:
            from mujoco import viewer

            vis = viewer.launch_passive(model, data)
            vis.user_scn.flags[int(mujoco.mjtRndFlag.mjRND_SHADOW)] = 0
            vis.user_scn.flags[int(mujoco.mjtRndFlag.mjRND_REFLECTION)] = 0
            vis.sync()
            self._viewer = vis
            self._next_viewer_sync = 0.0
            logger.info("MuJoCo 창 제목: Nero_orca_view  (녹화 중 손가락이 여기서 움직임)")
        except Exception:
            logger.exception("MuJoCo viewer failed to open; recording continues offscreen")
            self._viewer = None

    def _sync_viewer(self) -> None:
        vis = self._viewer
        if vis is None:
            return
        try:
            if not vis.is_running():
                self._viewer = None
                return
            now = time.perf_counter()
            if now < self._next_viewer_sync:
                return
            vis.sync()
            self._next_viewer_sync = now + 1.0 / self._viewer_hz
        except Exception:
            logger.exception("MuJoCo viewer sync failed")
            self._viewer = None

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
        for spec_name in NERO_JOINT_IDS:
            act_name = f"{spec_name}_act"
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, spec_name.replace("nero_", ""))
            if aid < 0 or jid < 0:
                raise RuntimeError(f"missing Nero actuator/joint for {spec_name}")
            self._nero_act_id.append(aid)
            self._nero_qpos_adr.append(int(model.jnt_qposadr[jid]))
        self._dataset_joint_ids = NERO_JOINT_IDS + self._orca_joint_ids

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

    def update_arm_hint(self, landmarks: object) -> None:
        self._arm_hints.update_from_landmarks(landmarks)

    def go_home(self) -> None:
        assert self._model is not None and self._data is not None
        with self._lock:
            self._arm_hints.reset()
            apply_rest_pose(self._model, self._data)
            if self._arm_ik is not None:
                self._arm_ik.capture_rest_ee(self._data)
            self._nero_cmd = np.array(REST_QPOS, dtype=np.float64)
            self._last_hand = self.home_position()
            self._write_hand_ctrl(self._last_hand)
            for _ in range(int(round(0.5 / self._timestep))):
                step_nero(self._model, self._data, nero_ctrl(self._nero_cmd))
            self._last_step_time = time.perf_counter()

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
            obs = SinkObservation(
                joint_state=self._read_joint_state(),
                images={self._camera_config.name: self._render(), **self._cameras.capture()},
            )
            self._sync_viewer()
            return obs

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

    def _write_hand_ctrl(self, action: OrcaJointPositions) -> None:
        assert self._data is not None
        for jid, aid in self._orca_act_id.items():
            if jid == "wrist":
                self._data.ctrl[aid] = 0.0
                continue
            deg = float(action.data.get(jid, 0.0))
            self._data.ctrl[aid] = math.radians(deg)

    def dispatch_action(self, action: OrcaJointPositions) -> None:
        assert self._model is not None and self._data is not None
        with self._lock:
            self._last_hand = action
            self._write_hand_ctrl(action)
            if self._arm_ik is not None:
                self._nero_cmd = self._arm_ik.solve(self._data, self._arm_hints.snapshot())
            now = time.perf_counter()
            if self._last_step_time is None:
                nstep = max(1, int(round(1.0 / (self._control_hz * self._timestep))))
            else:
                nstep = int(round((now - self._last_step_time) / self._timestep))
                nstep = int(np.clip(nstep, 1, MAX_SUBSTEPS))
            self._last_step_time = now
            hold = nero_ctrl(self._nero_cmd)
            for _ in range(nstep):
                step_nero(self._model, self._data, hold)

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
                logger.exception("combined sim step failed")
                break
            ticker.tick()

    def close(self) -> None:
        if self._model is None:
            return
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
            logger.exception("Combined SimSink close failed")
        finally:
            self._model = None
            self._data = None
            self._renderer = None
            self._record_camera = None
            self._viewer = None


def check_sink() -> None:
    sink = CombinedNeroOrcaSimSink(show_viewer=False)
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
        raise SystemExit(f"arm drifted off rest pose without a wrist hint ({rest_err:.2f} deg)")

    origin = ArmHint(wrist_image=np.array([0.50, 0.50, 0.12], dtype=np.float64))
    moved = ArmHint(wrist_image=np.array([0.78, 0.28, 0.20], dtype=np.float64))
    sink._arm_hints.update_from_landmarks(type("L", (), {"wrist_image": origin.wrist_image})())
    sink.dispatch_action(OrcaJointPositions(flexed))
    sink._arm_hints.update_from_landmarks(type("L", (), {"wrist_image": moved.wrist_image})())
    for _ in range(12):
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
    img = obs.images["frontal"]
    if img.ndim != 3 or img.shape[2] != 3:
        raise SystemExit(f"bad render shape {img.shape}")
    sink.close()
    print("ok: combined sink tracks fingers and moves Nero from wrist-image IK")


def main() -> None:
    parser = argparse.ArgumentParser(description="Combined Nero+Orca sim sink")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    if args.check:
        check_sink()
        return
    parser.error("pass --check, or use orca_teleop record_dataset.py --backend combined")


if __name__ == "__main__":
    main()
