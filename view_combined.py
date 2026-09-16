#!/usr/bin/env python3
"""View the combined Nero + Orca model, or check named finger control.

The viewer leaves ``data.ctrl`` alone so MuJoCo Control sliders work.
Arm joints still go through ``step_nero`` (slew + qvel clip).

    conda activate orca
    python attach_orca.py
    python view_combined.py
    python view_combined.py --check
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
from mujoco import viewer

from limits import apply_rest_pose, rest_ctrl, step_nero

ROOT = Path(__file__).resolve().parent
COMBINED_XML = ROOT / "models" / "nero_orca.xml"
SCENE_XML = ROOT / "models" / "nero_orca_scene.xml"

ARM_HOLD = rest_ctrl()
INDEX_MCP = "orca_right_i-mcp_actuator"


def load_model(path: Path) -> tuple[mujoco.MjModel, mujoco.MjData]:
    if not path.exists():
        raise FileNotFoundError(f"run attach_orca.py first: missing {path}")
    model = mujoco.MjModel.from_xml_path(str(path))
    return model, mujoco.MjData(model)


def actuator_id(model: mujoco.MjModel, name: str) -> int:
    aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
    if aid < 0:
        raise ValueError(f"actuator 이름을 확인하세요: {name}")
    return aid


def joint_qpos(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str) -> float:
    jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    if jid < 0:
        raise ValueError(f"joint 이름을 확인하세요: {joint_name}")
    adr = int(model.jnt_qposadr[jid])
    return float(data.qpos[adr])


def check_control(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    tower = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "orca_right_tower")
    if tower < 0:
        raise SystemExit("orca_right_tower missing")
    parent = model.body(int(model.body_parentid[tower])).name
    if parent != "link7":
        raise SystemExit(f"tower is parented to {parent}, expected link7")
    bodies = [model.body(i).name for i in range(model.nbody)]
    if not any("ForeArm" in name for name in bodies):
        raise SystemExit("forearm tower is missing from the model")
    wrist_act = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "orca_right_wrist_actuator")
    if wrist_act < 0:
        raise SystemExit("orca_right_wrist_actuator missing")
    carpals = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "orca_right_R-Carpals_8d1f1041")
    if carpals < 0:
        raise SystemExit("carpals body missing")
    carpals_parent = model.body(int(model.body_parentid[carpals])).name
    if "TopTower" not in carpals_parent:
        raise SystemExit(f"carpals parent is {carpals_parent}, expected TopTower")

    mujoco.mj_resetData(model, data)
    apply_rest_pose(model, data)
    rest = [ARM_HOLD[f"nero_joint{i}_act"] for i in range(1, 8)]
    for _ in range(2500):
        step_nero(model, data, ARM_HOLD)
    arm = [float(data.qpos[i]) for i in range(7)]
    print("hold arm qpos", [round(v, 4) for v in arm])
    hold_err = max(abs(a - b) for a, b in zip(arm, rest))
    if hold_err > 0.15:
        raise SystemExit(f"arm did not hold rest pose under hand mass: {arm}")

    aid = actuator_id(model, INDEX_MCP)
    data.ctrl[aid] = 0.8
    for _ in range(2500):
        step_nero(model, data, ARM_HOLD)
    finger = joint_qpos(model, data, "orca_right_i-mcp")
    arm_after = [float(data.qpos[i]) for i in range(7)]
    print(f"{INDEX_MCP} target=0.8  qpos={finger:.4f}")
    print("arm after finger", [round(v, 4) for v in arm_after])
    if abs(finger - 0.8) > 0.15:
        raise SystemExit(f"index mcp did not track 0.8 rad (qpos={finger:.4f})")
    drift = max(abs(a - b) for a, b in zip(arm, arm_after))
    if drift > 0.05:
        raise SystemExit(f"arm pose drifted while moving fingers ({drift:.4f})")
    print("ok: tower is a child of link7; fingers track by name; rest pose held")


def finger_mcp_ids(model: mujoco.MjModel) -> list[int]:
    ids = []
    for i in range(model.nu):
        name = model.actuator(i).name
        if name.startswith("orca_") and "-mcp_actuator" in name:
            ids.append(i)
    return ids


RENDER_HZ = 15.0


def apply_startup_render_flags(vis) -> None:
    """Default: shadows/reflections off. Viewer Rendering checkboxes can turn them back on."""
    vis.user_scn.flags[int(mujoco.mjtRndFlag.mjRND_SHADOW)] = 0
    vis.user_scn.flags[int(mujoco.mjtRndFlag.mjRND_REFLECTION)] = 0


def run_viewer(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    apply_rest_pose(model, data)
    render_period = 1.0 / RENDER_HZ
    print(
        "MuJoCo 창 제목: Nero_orca_view  (Control로 조종. Shadow/Reflection은 처음에만 꺼짐)",
        flush=True,
    )
    with viewer.launch_passive(model, data) as vis:
        apply_startup_render_flags(vis)
        next_sync = time.time()
        while vis.is_running():
            loop_start = time.time()
            for _ in range(8):
                step_nero(model, data)
            now = time.time()
            if now >= next_sync:
                vis.sync()
                next_sync = now + render_period
            leftover = render_period - (time.time() - loop_start)
            if leftover > 0:
                time.sleep(leftover)

def print_orca_mass(model):
    total = 0.0

    print("\n=== ORCA BODY MASS ===")

    for i in range(model.nbody):
        name = model.body(i).name

        if name.startswith("orca_"):
            mass = float(model.body_mass[i])
            total += mass

            print(
                f"{name:55s} "
                f"{mass:.6f} kg"
            )

    print("----------------------------")
    print(f"ORCA TOTAL MASS = {total:.4f} kg")


def main() -> None:
    parser = argparse.ArgumentParser(description="View or check Nero+Orca MJCF")
    parser.add_argument("--check", action="store_true", help="hold arm and track a named finger command")
    args = parser.parse_args()

    model, data = load_model(COMBINED_XML if args.check else SCENE_XML)
    print("bodies", [model.body(i).name for i in range(model.nbody)])
    print("actuators", [model.actuator(i).name for i in range(model.nu)])
    if args.check:
        check_control(model, data)
        return
    run_viewer(model, data)
    print_orca_mass(model)


if __name__ == "__main__":
    main()
