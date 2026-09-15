"""Nero joint limits: ROS/SDK source, slightly narrower sim ranges, velocity caps.

Sim angle limits sit 1 deg inside the pyAgxArm/ROS software clamp so a command
that is legal here is legal on the real arm. Velocity comes from the URDF
``velocity="5"`` (rad/s) and is enforced on ``data.qvel`` plus control slew.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

ANGLE_MARGIN_DEG = 1.0
ANGLE_MARGIN_RAD = math.radians(ANGLE_MARGIN_DEG)

# pyAgxArm Nero default joint_limits (rad). URDF rounded these down.
ROS_JOINTS = [
    {"joint": "joint1", "lower": -2.705261, "upper": 2.705261, "effort": 100.0, "velocity": 5.0},
    {"joint": "joint2", "lower": -1.745330, "upper": 1.745330, "effort": 100.0, "velocity": 5.0},
    {"joint": "joint3", "lower": -2.757621, "upper": 2.757621, "effort": 100.0, "velocity": 5.0},
    {"joint": "joint4", "lower": -1.012291, "upper": 2.146755, "effort": 100.0, "velocity": 5.0},
    {"joint": "joint5", "lower": -2.757621, "upper": 2.757621, "effort": 100.0, "velocity": 5.0},
    {"joint": "joint6", "lower": -0.733039, "upper": 0.959932, "effort": 100.0, "velocity": 5.0},
    {"joint": "joint7", "lower": -1.570797, "upper": 1.570797, "effort": 100.0, "velocity": 5.0},
]


def _sim_joint(spec: dict) -> dict:
    return {
        "joint": spec["joint"],
        "ros_lower": spec["lower"],
        "ros_upper": spec["upper"],
        "sim_lower": spec["lower"] + ANGLE_MARGIN_RAD,
        "sim_upper": spec["upper"] - ANGLE_MARGIN_RAD,
        "effort": spec["effort"],
        "ros_velocity": spec["velocity"],
        "sim_velocity": spec["velocity"],
    }


JOINTS = [_sim_joint(spec) for spec in ROS_JOINTS]
JOINT_BY_NAME = {spec["joint"]: spec for spec in JOINTS}

LIMITS_PATH = Path(__file__).resolve().parent / "models" / "nero_limits.json"


def actuator_to_joint(actuator_name: str) -> str:
    if actuator_name.startswith("nero_") and actuator_name.endswith("_act"):
        return actuator_name[len("nero_") : -len("_act")]
    return actuator_name


def dump_limits(path: Path, kp: float, kv: float) -> None:
    payload = {
        "angle_margin_deg": ANGLE_MARGIN_DEG,
        "note": (
            "sim angle range is ROS/SDK minus 1 deg per side; "
            "sim_velocity is URDF velocity (rad/s), enforced on qvel and ctrl slew"
        ),
        "kp": kp,
        "kv": kv,
        "joints": JOINTS,
    }
    path.write_text(json.dumps(payload, indent=2) + "\n")


def clip_qvel(model, data) -> None:
    for i in range(model.njnt):
        spec = JOINT_BY_NAME.get(model.joint(i).name)
        if spec is None:
            continue
        vmax = spec["sim_velocity"]
        data.qvel[i] = float(np.clip(data.qvel[i], -vmax, vmax))


def slew_ctrl(model, data, targets: dict[str, float] | None = None) -> None:
    """Move each actuator command toward target at most sim_velocity * dt."""
    dt = float(model.opt.timestep)
    for i in range(model.nu):
        act_name = model.actuator(i).name
        spec = JOINT_BY_NAME.get(actuator_to_joint(act_name))
        if spec is None:
            continue
        vmax = spec["sim_velocity"]
        lo, hi = model.actuator_ctrlrange[i]
        current = float(data.ctrl[i])
        target = float(targets.get(act_name, current)) if targets else current
        target = float(np.clip(target, lo, hi))
        data.ctrl[i] = current + float(np.clip(target - current, -vmax * dt, vmax * dt))


def step_nero(model, data, targets: dict[str, float] | None = None) -> None:
    slew_ctrl(model, data, targets)
    import mujoco

    mujoco.mj_step(model, data)
    clip_qvel(model, data)
