#!/usr/bin/env python3
"""Attach the official Orca v2 right hand (tower + wrist + fingers) to Nero link7.

The real hardware mounts the forearm tower together with the hand, so the
full official body tree is attached — not the carpal subtree alone. Orca's
``right_wrist`` joint is kept. Mount pose starts from the AgileX Revo2 flange
plus a +90 deg X rotation so the palm sits distal to the tower.

    conda activate orca
    python attach_orca.py
"""

from __future__ import annotations

import math
import os
import re
from pathlib import Path

import mujoco

from limits import JOINTS

ROOT = Path(__file__).resolve().parent
MODELS = ROOT / "models"
NERO_XML = MODELS / "nero.xml"
HAND_XML = MODELS / "orca_right_hand.xml"
COMBINED_XML = MODELS / "nero_orca.xml"
SCENE_XML = MODELS / "nero_orca_scene.xml"

ORCA_V2 = ROOT.parent / "orca_sim" / "src" / "orca_sim" / "models" / "v2"

# AgileX Revo2: link7 -> flange. Extra +90 deg about X maps Orca tower +Y
# (along-hand in the official MJCF) onto the flange +Z (tool axis).
# Z is +90 deg, not 180: Nero joint4 folds more toward world -X at home, so
# the palm faces that inner-elbow side and the knuckles stay parallel to the
# joint4 hinge (world Y).
FLANGE_POS = [0.031, 0.0, -0.0235]
FLANGE_EULER_XYZ = [-math.pi / 2, 0.0, -math.pi / 2]
HAND_POS = [0.0, 0.0, 0.012]
HAND_EULER_XYZ = [math.pi / 2, 0.0, math.pi / 2]

HAND_PREFIX = "orca_"
TOWER = f"{HAND_PREFIX}right_tower"
FOREARM = f"{HAND_PREFIX}right_ForeArmStructure-Model_e18f2368"
TOP_TOWER = f"{HAND_PREFIX}right_TopTower-Model_4a80d30e"
CARPALS = f"{HAND_PREFIX}right_R-Carpals_8d1f1041"
WRIST_ACTUATOR = f"{HAND_PREFIX}right_wrist_actuator"

ARM_KP = 400.0
ARM_KV = 40.0

EXCLUDE_PAIRS = [
    ("link5", FOREARM),
    ("link6", FOREARM),
    ("link7", FOREARM),
    ("link5", TOP_TOWER),
    ("link6", TOP_TOWER),
    ("link7", TOP_TOWER),
    ("link6", CARPALS),
    ("link7", CARPALS),
]

NERO_ACTUATOR_COUNT = 7
ORCA_ACTUATOR_COUNT = 17


def write_hand_wrapper() -> None:
    if not ORCA_V2.exists():
        raise FileNotFoundError(f"missing official Orca v2 models: {ORCA_V2}")
    rel = Path(os.path.relpath(ORCA_V2.resolve(), MODELS.resolve())).as_posix()
    HAND_XML.write_text(
        f"""<mujoco model="orca_right_hand">
  <!-- Full official v2 tree: tower + wrist joint + fingers. No floor. -->
  <include file="{rel}/assets/options.xml"/>
  <include file="{rel}/mjcf/orcahand_right.mjcf"/>
  <worldbody>
    <include file="{rel}/mjcf/orcahand_right_body.xml"/>
  </worldbody>
</mujoco>
"""
    )


def relativize_mesh_paths(xml: str) -> str:
    abs_orca = (ORCA_V2 / "assets" / "right").resolve().as_posix()
    rel_orca = Path(os.path.relpath(ORCA_V2 / "assets" / "right", MODELS)).as_posix()
    xml = xml.replace(abs_orca + "/", rel_orca + "/")
    xml = xml.replace(abs_orca, rel_orca)
    if 'timestep="' not in xml:
        xml = xml.replace(
            '<option integrator="implicitfast"/>',
            '<option integrator="implicitfast" timestep="0.002"/>',
            1,
        )
    xml = re.sub(r'meshdir="\./"', 'meshdir="."', xml, count=1)
    return xml


def write_scene(path: Path) -> None:
    path.write_text(
        """<mujoco model="nero_orca_view">
  <include file="nero_orca.xml"/>
  <statistic center="0 0 0.45" extent="1.3"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <global azimuth="140" elevation="-20"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
"""
    )


def attach() -> mujoco.MjSpec:
    if not NERO_XML.exists():
        raise FileNotFoundError(f"run prepare_nero.py first: missing {NERO_XML}")
    write_hand_wrapper()

    parent = mujoco.MjSpec.from_file(str(NERO_XML))
    parent.compiler.eulerseq = "XYZ"
    parent.option.timestep = 0.002
    child = mujoco.MjSpec.from_file(str(HAND_XML))

    link7 = parent.body("link7")
    flange = link7.add_frame(
        name="revo2_flange", pos=FLANGE_POS, euler=FLANGE_EULER_XYZ
    )
    mount = flange.add_frame(name="orca_mount", pos=HAND_POS, euler=HAND_EULER_XYZ)
    parent.attach(child, prefix=HAND_PREFIX, frame=mount)

    for body1, body2 in EXCLUDE_PAIRS:
        parent.add_exclude(bodyname1=body1, bodyname2=body2)

    for spec in JOINTS:
        act = parent.actuator(f"nero_{spec['joint']}_act")
        act.set_to_position(kp=ARM_KP, kv=ARM_KV)
        act.ctrlrange = [spec["sim_lower"], spec["sim_upper"]]
        act.forcerange = [-spec["effort"], spec["effort"]]
        act.ctrllimited = True
        act.forcelimited = True
    return parent


def main() -> None:
    MODELS.mkdir(parents=True, exist_ok=True)
    spec = attach()
    spec.compile()
    spec.to_file(str(COMBINED_XML))
    COMBINED_XML.write_text(relativize_mesh_paths(COMBINED_XML.read_text()))
    write_scene(SCENE_XML)

    model = mujoco.MjModel.from_xml_path(str(COMBINED_XML))
    bodies = [model.body(i).name for i in range(model.nbody)]
    acts = [model.actuator(i).name for i in range(model.nu)]
    tower = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, TOWER)
    if tower < 0:
        raise RuntimeError(f"attached tower {TOWER} is missing")
    parent_id = int(model.body_parentid[tower])
    parent_name = model.body(parent_id).name
    if parent_name != "link7":
        raise RuntimeError(f"tower parent is {parent_name!r}, expected link7")
    if WRIST_ACTUATOR not in acts:
        raise RuntimeError(f"missing {WRIST_ACTUATOR}")
    if FOREARM not in bodies:
        raise RuntimeError("forearm tower body is missing")
    nero_acts = [n for n in acts if n.startswith("nero_")]
    orca_acts = [n for n in acts if n.startswith(HAND_PREFIX)]
    print(f"wrote {HAND_XML}")
    print(f"wrote {COMBINED_XML}")
    print(f"wrote {SCENE_XML}")
    print(f"bodies ({model.nbody}): {bodies}")
    print(f"nero actuators ({len(nero_acts)}): {nero_acts}")
    print(f"orca actuators ({len(orca_acts)}): {orca_acts}")
    print(f"tower {TOWER} parent={parent_name}")
    if len(nero_acts) != NERO_ACTUATOR_COUNT:
        raise RuntimeError(f"expected {NERO_ACTUATOR_COUNT} Nero actuators, got {len(nero_acts)}")
    if len(orca_acts) != ORCA_ACTUATOR_COUNT:
        raise RuntimeError(f"expected {ORCA_ACTUATOR_COUNT} Orca actuators, got {len(orca_acts)}")
    mujoco.MjModel.from_xml_path(str(SCENE_XML))


if __name__ == "__main__":
    main()
