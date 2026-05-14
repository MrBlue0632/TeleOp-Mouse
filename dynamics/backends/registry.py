"""Backend factory and capability notes."""

from __future__ import annotations

from typing import Any

from .base import RobotBackend
from .xarm import XArmBackend


CAPABILITIES = {
    "xarm": {
        "implemented": True,
        "position": "get_joint_states() or get_servo_angle()",
        "velocity": "get_joint_states() or realtime_joint_speeds",
        "torque": "get_joints_torque(), get_joint_states effort, or joints_torque report cache",
        "teach_mode": "set_mode(2)",
        "payload": "iden_tcp_load(), set_tcp_load(weight, center_of_gravity_mm)",
    },
    "ur": {
        "implemented": False,
        "position": "RTDE actual_q",
        "velocity": "RTDE actual_qd",
        "torque": "RTDE target_moment; currents via actual_current/target_current",
        "teach_mode": "freedrive mode through URScript/dashboard integration",
        "payload": "set_payload / installation payload parameters",
    },
    "franka": {
        "implemented": False,
        "position": "libfranka RobotState.q",
        "velocity": "libfranka RobotState.dq",
        "torque": "tau_J and tau_ext_hat_filtered",
        "teach_mode": "robot-mode specific guiding/brake release workflow",
        "payload": "RobotState load mass/COM/inertia and model API",
    },
    "kinova": {
        "implemented": False,
        "position": "Kortex BaseCyclic feedback actuator position",
        "velocity": "Kortex BaseCyclic feedback actuator velocity",
        "torque": "Kortex actuator torque/current feedback where supported",
        "teach_mode": "actuator/base admittance or hand-guiding mode depending on model",
        "payload": "model-specific payload configuration",
    },
}


def robot_capabilities(robot: str) -> dict[str, Any]:
    key = robot.lower()
    if key in {"xarm6", "xarm7"}:
        key = "xarm"
    return CAPABILITIES.get(key, {"implemented": False})


def create_backend(config: dict[str, Any], *, robot: str | None = None) -> RobotBackend:
    backend = (config.get("sdk_backend") or robot or config.get("robot_name") or "").lower()
    if backend in {"xarm", "xarm6", "xarm7"}:
        return XArmBackend.from_config(config)
    caps = robot_capabilities(backend)
    raise NotImplementedError(
        f"robot backend '{backend}' is not implemented yet. Capability notes: {caps}"
    )
