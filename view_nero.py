#!/usr/bin/env python3
"""Open the prepared Nero MJCF in the MuJoCo viewer, or run a hold/track check.

    conda activate orca
    python view_nero.py
    python view_nero.py --check
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
from mujoco import viewer

from limits import JOINT_BY_NAME, clip_qvel, step_nero

ROOT = Path(__file__).resolve().parent
NERO_XML = ROOT / "models" / "nero.xml"
SCENE_XML = ROOT / "models" / "nero_scene.xml"


def load_model(path: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    if not path.exists():
        raise FileNotFoundError(f"run prepare_nero.py first: missing {path}")
    model = mujoco.MjModel.from_xml_path(str(path))
    return model, mujoco.MjData(model)


def actuator_id(model: mujoco.MjModel, name: str) -> int:
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if aid < 0:
        raise ValueError(f"NERO actuator 이름을 확인하세요: {name}")
    return aid


def check_control(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Small named command, plus a large move that must stay under sim vmax."""
    mujoco.mj_resetData(model, data)
    aid = actuator_id(model, "nero_joint1_act")
    target = 0.2
    for _ in range(2000):
        step_nero(model, data, {"nero_joint1_act": target})
    q = float(data.qpos[0])
    v = float(data.qvel[0])
    print(f"nero_joint1_act target={target:.3f}  qpos={q:.4f}  qvel={v:.4f}")
    if abs(q - target) > 0.05:
        raise SystemExit(f"joint1 did not track 0.2 rad (qpos={q:.4f})")

    mujoco.mj_resetData(model, data)
    far = float(JOINT_BY_NAME["joint1"]["sim_upper"])
    vmax = float(JOINT_BY_NAME["joint1"]["sim_velocity"])
    peak = 0.0
    for _ in range(4000):
        step_nero(model, data, {"nero_joint1_act": far})
        peak = max(peak, abs(float(data.qvel[0])))
    print(f"large move peak |qvel|={peak:.4f}  sim_vmax={vmax:g}")
    if peak > vmax + 1e-6:
        raise SystemExit(f"joint1 exceeded sim velocity ({peak:.4f} > {vmax:g})")
    print("ok: named actuator tracked a small angle; qvel stayed within sim vmax")


def run_viewer(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    with viewer.launch_passive(model, data) as vis:
        while vis.is_running():
            mujoco.mj_step(model, data)
            clip_qvel(model, data)
            vis.sync()


def main() -> None:
    parser = argparse.ArgumentParser(description="View or check Nero MJCF")
    parser.add_argument("--check", action="store_true", help="track a small joint1 command without opening the viewer")
    args = parser.parse_args()

    model, data = load_model(NERO_XML if args.check else SCENE_XML)
    print("bodies", [model.body(i).name for i in range(model.nbody)])
    print("actuators", [model.actuator(i).name for i in range(model.nu)])
    if args.check:
        check_control(model, data)
        return
    run_viewer(model, data)


if __name__ == "__main__":
    main()
