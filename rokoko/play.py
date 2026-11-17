"""Step 4: drive the Orca hand in the MuJoCo scene from Rokoko glove data (sim only).

    python rokoko/play.py --file rokoko/samples/hand_20261116_160126.npz   # recorded playback
    python rokoko/play.py --live                                            # UDP stream, port 14043

Playback replays frames at their recorded timing (``--speed`` scales it, ``--loop`` repeats).
Live mode keeps only the newest queued packet per step. The Nero arm stays at its rest pose.
Hold the hand open and flat for the first second: that window calibrates the hand scale
(restart to recalibrate).
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / "orca_teleop" / "src"))

import mujoco  # noqa: E402
import mujoco.viewer  # noqa: E402
from orca_teleop.sim import _sim_joint_to_config_id  # noqa: E402

from combined_sink import _orca_core_config_path  # noqa: E402
from limits import apply_rest_pose  # noqa: E402
from rokoko.retarget import DEFAULT_CALIB_FILE, RokokoHandRetargeter  # noqa: E402
from rokoko.stream import DEFAULT_PORT, RokokoReceiver, frame_to_positions  # noqa: E402

SCENE_XML = ROOT / "models" / "nero_orca_scene.xml"
URDF = ROOT.parent / "orcahand_description" / "v1" / "models" / "urdf" / "orcahand_right.urdf"
HAND_PREFIX = "orca_right_"


def map_orca_actuators(model: mujoco.MjModel, joint_ids: list[str]) -> dict[str, int]:
    valid = set(joint_ids)
    act: dict[str, int] = {}
    for i in range(model.nu):
        name = model.actuator(i).name
        if name.startswith("nero_"):
            continue
        jnt = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(model.actuator_trnid[i, 0])) or ""
        core = jnt[len(HAND_PREFIX):] if jnt.startswith(HAND_PREFIX) else jnt
        cid = _sim_joint_to_config_id(core, valid)
        if cid is None:
            raise ValueError(f"no orca_core joint for actuator {name} (joint {jnt})")
        act[cid] = i
    missing = valid - set(act)
    if missing:
        raise ValueError(f"sim has no actuator for {sorted(missing)}")
    return act


def main() -> None:
    ap = argparse.ArgumentParser()
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--file", help="recorded .npz from rokoko/record.py")
    src.add_argument("--live", action="store_true", help="read the UDP stream")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT)
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed factor")
    ap.add_argument("--loop", action="store_true")
    ap.add_argument("--config-path", default=None, help="adaptive_analytical yaml override")
    ap.add_argument("--smoothing", type=float, default=0.5)
    ap.add_argument("--max-step-deg", type=float, default=12.0)
    ap.add_argument("--calibrate", action="store_true",
                    help="guided 3-pose glove calibration after the open-hand second; saved to --calib-file")
    ap.add_argument("--calib-file", default=str(DEFAULT_CALIB_FILE),
                    help="operator calibration JSON to load (or write with --calibrate); '' = built-in constants")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(str(SCENE_XML))
    data = mujoco.MjData(model)
    apply_rest_pose(model, data)

    retargeter = RokokoHandRetargeter(
        str(_orca_core_config_path("right")), str(URDF),
        config_path=args.config_path, smoothing=args.smoothing, max_step_deg=args.max_step_deg,
        calibrate=args.calibrate, calib_file=args.calib_file or None, say=print,
    )
    act = map_orca_actuators(model, retargeter.joint_ids)
    print("joints:", retargeter.joint_ids)

    if args.file:
        d = np.load(args.file)
        frames, times = d["positions"].astype(float), d["t"] - d["t"][0]
        print(f"playing {len(frames)} frames, {times[-1]:.1f}s, speed x{args.speed}")
    else:
        rx = RokokoReceiver(args.port)
        print(f"live on 0.0.0.0:{args.port}; hold the hand open for the first second")

    def apply(positions: np.ndarray, t: float | None = None) -> bool:
        q = retargeter(positions, t)
        if q is None:
            return False
        for jid, deg in zip(retargeter.joint_ids, retargeter.joint_array(q), strict=True):
            data.ctrl[act[jid]] = math.radians(float(deg))
        return True

    n_sub = max(1, int(round((1 / 30) / model.opt.timestep)))
    with mujoco.viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=False) as v:
        v.cam.lookat[:] = data.body("orca_right_tower").xpos if model.body("orca_right_tower").id >= 0 else data.body(1).xpos
        v.cam.distance, v.cam.azimuth, v.cam.elevation = 0.6, 160, -25
        last_log = time.time()
        n_cmd = 0
        while v.is_running():
            if args.file:
                t0 = time.time()
                i = 0
                while i < len(frames) and v.is_running():
                    target = t0 + times[i] / args.speed
                    now = time.time()
                    if now < target:
                        time.sleep(min(target - now, 0.01))
                        continue
                    n_cmd += apply(frames[i], float(times[i]))
                    i += 1
                    for _ in range(n_sub):
                        mujoco.mj_step(model, data)
                    v.sync()
                if not args.loop:
                    break
                retargeter.reset()
            else:
                frame = rx.latest()
                if frame is not None:
                    pos = frame_to_positions(frame)
                    if pos is not None:
                        n_cmd += apply(pos)
                for _ in range(n_sub):
                    mujoco.mj_step(model, data)
                v.sync()
                if time.time() - last_log > 5:
                    print(f"  {n_cmd} hand commands so far")
                    last_log = time.time()
                time.sleep(0.001)
        if args.file:
            print(f"done: {n_cmd} commands applied")
            while v.is_running():
                mujoco.mj_step(model, data)
                v.sync()
                time.sleep(0.01)


if __name__ == "__main__":
    main()
