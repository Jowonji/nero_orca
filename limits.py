"""Nero joint limits: ROS/SDK source, slightly narrower sim ranges, velocity caps.

Sim angle limits sit 1 deg inside the pyAgxArm/ROS software clamp so a command
that is legal here is legal on the real arm. Velocity comes from the URDF
``velocity="5"`` (rad/s) and is enforced on ``data.qvel`` plus control slew.

``effort`` is the pyAgxArm MIT feed-forward torque clamp (N·m), not the
placeholder ``100`` in the official URDF. Actuator ``forcerange`` uses this
so the sim saturates like the real motors.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

ANGLE_MARGIN_DEG = 1.0
ANGLE_MARGIN_RAD = math.radians(ANGLE_MARGIN_DEG)

# pyAgxArm Nero default joint_limits (rad). URDF rounded these down.
# effort: MIT t_ff clamp from pyAgxArm Nero driver (joints 1-2 / 3-4 / 5-7).
ROS_JOINTS = [
    {"joint": "joint1", "lower": -2.705261, "upper": 2.705261, "effort": 24.0, "velocity": 5.0},
    {"joint": "joint2", "lower": -1.745330, "upper": 1.745330, "effort": 24.0, "velocity": 5.0},
    {"joint": "joint3", "lower": -2.757621, "upper": 2.757621, "effort": 16.0, "velocity": 5.0},
    {"joint": "joint4", "lower": -1.012291, "upper": 2.146755, "effort": 16.0, "velocity": 5.0},
    {"joint": "joint5", "lower": -2.757621, "upper": 2.757621, "effort": 8.0, "velocity": 5.0},
    {"joint": "joint6", "lower": -0.733039, "upper": 0.959932, "effort": 8.0, "velocity": 5.0},
    {"joint": "joint7", "lower": -1.570797, "upper": 1.570797, "effort": 8.0, "velocity": 5.0},
]

# AgileX Isaac Lab / teleop init pose. Not URDF zeros.
REST_QPOS = [0.0, 0.0, 0.0, 1.22, 0.0, 0.0, 1.31]


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


def nero_ctrl(q) -> dict[str, float]:
    q = np.asarray(q, dtype=np.float64).reshape(-1)
    return {f"nero_{spec['joint']}_act": float(q[i]) for i, spec in enumerate(JOINTS)}


def rest_ctrl() -> dict[str, float]:
    return nero_ctrl(REST_QPOS)


def apply_rest_pose(model, data) -> None:
    """Put Nero joints and their actuators at the official rest pose."""
    import mujoco

    for spec, q in zip(JOINTS, REST_QPOS):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, spec["joint"])
        if jid >= 0:
            data.qpos[int(model.jnt_qposadr[jid])] = q
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"nero_{spec['joint']}_act")
        if aid >= 0:
            data.ctrl[aid] = q
    mujoco.mj_forward(model, data)


def actuator_to_joint(actuator_name: str) -> str:
    if actuator_name.startswith("nero_") and actuator_name.endswith("_act"):
        return actuator_name[len("nero_") : -len("_act")]
    return actuator_name


def dump_limits(path: Path, kp: float, kv: float) -> None:
    payload = {
        "angle_margin_deg": ANGLE_MARGIN_DEG,
        "note": (
            "sim angle range is ROS/SDK minus 1 deg per side; "
            "sim_velocity is URDF velocity (rad/s), enforced on qvel and ctrl slew; "
            "effort is pyAgxArm MIT t_ff clamp (N·m)"
        ),
        "rest_qpos": REST_QPOS,
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
