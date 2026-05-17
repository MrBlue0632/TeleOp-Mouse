"""Standalone robot dynamics and calibration tools."""

from .embodiment import EmbodimentMismatchError, assert_embodiment_matches, build_embodiment_metadata
from .resolver import PayloadSpec, ResolvedJoint, ResolvedLink, ResolvedRobot, resolve_robot

_API_EXPORTS = {
    "DEFAULT_COMPENSATION_PATH",
    "DEFAULT_TORQUE_DIR",
    "DEFAULT_TRAJ_DIR",
    "MODES",
    "TRAJ_KINDS",
    "build_theoretical_model",
    "load_dynamics_config",
    "resolve_robot_from_config",
    "run_cli_args",
    "run_mode",
}


def __getattr__(name: str):
    if name in _API_EXPORTS:
        from . import api

        return getattr(api, name)
    raise AttributeError(f"module 'dynamics' has no attribute {name!r}")


__all__ = [
    "DEFAULT_COMPENSATION_PATH",
    "DEFAULT_TORQUE_DIR",
    "DEFAULT_TRAJ_DIR",
    "MODES",
    "TRAJ_KINDS",
    "PayloadSpec",
    "ResolvedJoint",
    "ResolvedLink",
    "ResolvedRobot",
    "EmbodimentMismatchError",
    "assert_embodiment_matches",
    "build_embodiment_metadata",
    "build_theoretical_model",
    "load_dynamics_config",
    "resolve_robot_from_config",
    "run_cli_args",
    "run_mode",
    "resolve_robot",
]
