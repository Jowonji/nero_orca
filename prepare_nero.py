#!/usr/bin/env python3
"""Prepare AgileX Nero URDF for MuJoCo and write MJCF with position actuators.

Reads the official arm-only URDF (no gripper xacro), rewrites ROS mesh paths
to the existing STL files, compiles to MJCF, then adds one named position
actuator per revolute joint.

Visual meshes in the upstream URDF are Collada .dae with per-face materials.
This MuJoCo build cannot decode DAE, so collision stays on the official STL
and visuals are split from the DAE by material color.
"""

from __future__ import annotations

import os
import re
from pathlib import Path

import mujoco

from limits import JOINTS, dump_limits
from dae_visuals import convert_all

ROOT = Path(__file__).resolve().parent
MODELS = ROOT / "models"
UPSTREAM_URDF = ROOT.parent / "agx_arm_urdf" / "nero" / "urdf" / "nero_description.urdf"
MESH_DIR = ROOT.parent / "agx_arm_urdf" / "nero" / "meshes"

PACKAGE_DAE = "package://agx_arm_description/agx_arm_urdf/nero/meshes/dae/"
PACKAGE_MESH = "package://agx_arm_description/agx_arm_urdf/nero/meshes/"

KP = 100.0
KV = 10.0


def build_prepared_urdf() -> str:
    if not UPSTREAM_URDF.exists():
        raise FileNotFoundError(f"missing official URDF: {UPSTREAM_URDF}")
    if not MESH_DIR.exists():
        raise FileNotFoundError(f"missing Nero meshes: {MESH_DIR}")

    text = UPSTREAM_URDF.read_text()
    missing = sorted(
        {
            name
            for name in re.findall(r"meshes/(?:dae/)?([^\"']+\.dae)", text)
            for name in [name.replace(".dae", ".stl")]
            if not (MESH_DIR / name).exists()
        }
    )
    if missing:
        raise FileNotFoundError(f"STL meshes not found in {MESH_DIR}: {missing}")

    rel_mesh = Path(os.path.relpath(MESH_DIR.resolve(), MODELS.resolve()))
    text = text.replace(PACKAGE_DAE, str(rel_mesh) + "/")
    text = text.replace(".dae", ".stl")
    text = text.replace(PACKAGE_MESH, str(rel_mesh) + "/")

    compiler = """
    <!-- MuJoCo compile hints: keep visuals, do not fuse fixed bodies (link7). -->
    <mujoco>
        <compiler discardvisual="false" fusestatic="false" strippath="false"/>
    </mujoco>
"""
    if "<mujoco>" not in text:
        text = text.replace('<robot name="nero">', '<robot name="nero">' + compiler, 1)
    return apply_sim_limits_to_urdf(text)


def apply_sim_limits_to_urdf(text: str) -> str:
    """Replace URDF <limit> with sim ranges (ROS/SDK minus 1 deg, URDF velocity)."""
    for spec in JOINTS:
        pattern = (
            rf'(<joint name="{spec["joint"]}"[\s\S]*?<limit )'
            r'lower="[^"]*" upper="[^"]*" effort="[^"]*" velocity="[^"]*"'
        )
        repl = (
            rf'\1lower="{spec["sim_lower"]:.10g}" upper="{spec["sim_upper"]:.10g}" '
            rf'effort="{spec["effort"]:g}" velocity="{spec["sim_velocity"]:g}"'
        )
        text, n = re.subn(pattern, repl, text, count=1)
        if n != 1:
            raise RuntimeError(f"failed to patch URDF limit for {spec['joint']}")
    return text


def _rel(target: Path, start: Path) -> str:
    return os.path.relpath(target.resolve(), start.resolve())


def add_position_actuators(xml: str) -> str:
    lines = [
        "  <actuator>",
        "    <!-- kp/kv are starting gains; tune after hold/vibration checks. -->",
    ]
    for spec in JOINTS:
        name = f"nero_{spec['joint']}_act"
        lines.append(
            "    <position"
            f' name="{name}"'
            f' joint="{spec["joint"]}"'
            ' gear="1"'
            f' kp="{KP:g}"'
            f' kv="{KV:g}"'
            ' ctrllimited="true"'
            f' ctrlrange="{spec["sim_lower"]:g} {spec["sim_upper"]:g}"'
            ' forcelimited="true"'
            f' forcerange="{-spec["effort"]:g} {spec["effort"]:g}"/>'
        )
    lines.append("  </actuator>")
    block = "\n".join(lines) + "\n"
    if "</mujoco>" not in xml:
        raise RuntimeError("compiled MJCF is missing </mujoco>")
    return xml.replace("</mujoco>", block + "</mujoco>", 1)


ADJACENT_BODIES = [
    ("base_link", "link1"),
    ("link1", "link2"),
    ("link2", "link3"),
    ("link3", "link4"),
    ("link4", "link5"),
    ("link5", "link6"),
    ("link6", "link7"),
]


def add_runtime_defaults(xml: str) -> str:
    """URDF compile leaves undamped joints and no adjacent-link excludes."""
    if "<option" not in xml:
        xml = xml.replace(
            "  <asset>",
            '  <option integrator="implicitfast" timestep="0.002"/>\n\n  <asset>',
            1,
        )
    xml = re.sub(
        r'(<joint name="joint[1-7]")',
        r'\1 damping="1" armature="0.05"',
        xml,
    )
    excludes = "\n".join(
        f'    <exclude body1="{a}" body2="{b}"/>' for a, b in ADJACENT_BODIES
    )
    xml = xml.replace(
        "  <actuator>",
        f"  <contact>\n{excludes}\n  </contact>\n  <actuator>",
        1,
    )
    return xml


def relativize_mesh_paths(xml: str) -> str:
    rel_dir = _rel(MESH_DIR, MODELS)
    xml = re.sub(r'file="(?:[^"]*/)?([^"/]+\.stl)"', rf'file="{rel_dir}/\1"', xml)
    if 'meshdir="' in xml:
        xml = re.sub(r'meshdir="[^"]*"', 'meshdir="."', xml, count=1)
    else:
        xml = xml.replace(
            '<compiler angle="radian"/>',
            '<compiler angle="radian" meshdir="." strippath="false"/>',
            1,
        )
    return xml


def apply_visual_colors(xml: str, visuals: dict[str, list[dict]]) -> str:
    extra_assets: list[str] = []
    for link, parts in visuals.items():
        geoms: list[str] = []
        for part in parts:
            r, g, b, a = part["rgba"]
            mat = f'{part["name"]}_mat'
            extra_assets.append(
                f'    <mesh name="{part["name"]}" content_type="model/stl" file="{part["file"]}"/>'
            )
            extra_assets.append(
                f'    <material name="{mat}" rgba="{r:.6g} {g:.6g} {b:.6g} {a:.6g}" specular="0.15" shininess="0.08"/>'
            )
            geoms.append(
                f'<geom type="mesh" contype="0" conaffinity="0" group="1" density="0" '
                f'mesh="{part["name"]}" material="{mat}"/>'
            )
        if not geoms:
            continue
        xml, n = re.subn(
            rf'<geom type="mesh" contype="0" conaffinity="0" group="1" density="0" mesh="{link}"/>',
            "\n      ".join(geoms),
            xml,
            count=1,
        )
        if n != 1:
            raise RuntimeError(f"failed to replace visual geom for {link}")
    xml = xml.replace("  </asset>", "\n".join(extra_assets) + "\n  </asset>", 1)
    xml = re.sub(
        r'<geom type="mesh" mesh="(base_link|link[1-7])"/>',
        r'<geom type="mesh" mesh="\1" group="4" rgba="0 0 0 0"/>',
        xml,
    )
    return xml


def write_view_scene(path: Path) -> None:
    path.write_text(
        """<mujoco model="nero_view">
  <include file="nero.xml"/>
  <statistic center="0 0 0.35" extent="1.2"/>
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


def main() -> None:
    MODELS.mkdir(parents=True, exist_ok=True)
    prepared_path = MODELS / "nero_prepared.urdf"
    xml_path = MODELS / "nero.xml"
    scene_path = MODELS / "nero_scene.xml"
    limits_path = MODELS / "nero_limits.json"

    visuals = convert_all()
    prepared_path.write_text(build_prepared_urdf())
    model = mujoco.MjModel.from_xml_path(str(prepared_path))
    mujoco.mj_saveLastXML(str(xml_path), model)

    xml = relativize_mesh_paths(xml_path.read_text())
    xml = apply_visual_colors(xml, visuals)
    xml = add_position_actuators(xml)
    xml = add_runtime_defaults(xml)
    xml_path.write_text(xml)
    write_view_scene(scene_path)

    dump_limits(limits_path, KP, KV)

    model = mujoco.MjModel.from_xml_path(str(xml_path))
    bodies = [model.body(i).name for i in range(model.nbody)]
    joints = [model.joint(i).name for i in range(model.njnt)]
    acts = [model.actuator(i).name for i in range(model.nu)]
    print(f"wrote {prepared_path}")
    print(f"wrote {xml_path}")
    print(f"wrote {scene_path}")
    print(f"bodies ({model.nbody}): {bodies}")
    print(f"joints ({model.njnt}): {joints}")
    print(f"actuators ({model.nu}): {acts}")
    for i in range(model.njnt):
        spec = JOINTS[i]
        print(
            f"  {model.joint(i).name} range={model.jnt_range[i]}  "
            f"act={acts[i]}  ctrlrange={model.actuator_ctrlrange[i]}  "
            f"vmax={spec['sim_velocity']:g} rad/s"
        )
    if "link7" not in bodies:
        raise RuntimeError("end-effector body link7 was fused away")
    if model.nu != 7:
        raise RuntimeError(f"expected 7 actuators, got {model.nu}")
    mujoco.MjModel.from_xml_path(str(scene_path))


if __name__ == "__main__":
    main()
