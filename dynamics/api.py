"""Public Python facade for the dynamics package."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence

from .config import load_config
from .main import DEFAULT_COMPENSATION_PATH, DEFAULT_TORQUE_DIR, DEFAULT_TRAJ_DIR, MODES, TRAJ_KINDS
from .resolver import ResolvedRobot, resolve_robot


def load_dynamics_config(
    config_path: str | Path | None = None,
    *,
    robot: str = "xarm6",
    overrides: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Load a robot dynamics config with optional overrides."""
    return load_config(config_path, robot=robot, overrides=overrides)


def resolve_robot_from_config(config: dict[str, Any]) -> ResolvedRobot:
    """Resolve URDF and payload metadata from a loaded dynamics config."""
    return resolve_robot(
        config["urdf_path"],
        name=str(config.get("robot_name", "robot")),
        payload=config.get("payload"),
    )


def build_theoretical_model(config: dict[str, Any], **kwargs: Any):
    """Build a theoretical dynamics model from a loaded config."""
    from .model import TheoreticalModel

    return TheoreticalModel(resolve_robot_from_config(config), **kwargs)


def run_mode(
    mode: str,
    config: dict[str, Any],
    *,
    output_dir: str | Path | None = None,
    duration_s: float | None = None,
    teach_sensitivity: int | None = None,
    traj_path: str | Path | Sequence[str | Path] | None = None,
    traj_kind: str | None = None,
    workspace_points: int | None = None,
    workspace_margin_ratio: float | None = None,
    workspace_speed_deg_s: float | None = None,
    workspace_seed: int | None = None,
    data_path: str | Path | None = None,
    static_data_path: str | Path | None = None,
    stop_data_path: str | Path | None = None,
    model_path: str | Path | None = None,
    model_kind: str | None = None,
    target: str | None = None,
    epochs: int | None = None,
    lr: float | None = None,
    hidden_dim: int | None = None,
    seed: int | None = None,
    static_alpha: float | None = None,
    speed_threshold_deg_s: float | None = None,
    motion_history_steps: int | None = None,
    hz: float | None = None,
) -> Path | list[Path] | None:
    """Run one dynamics workflow mode through the local orchestration module."""
    from .main import run_mode as run_workflow_mode

    return run_workflow_mode(
        mode,
        config,
        output_dir=output_dir,
        duration_s=duration_s,
        teach_sensitivity=teach_sensitivity,
        traj_path=traj_path,
        traj_kind=traj_kind,
        workspace_points=workspace_points,
        workspace_margin_ratio=workspace_margin_ratio,
        workspace_speed_deg_s=workspace_speed_deg_s,
        workspace_seed=workspace_seed,
        data_path=data_path,
        static_data_path=static_data_path,
        stop_data_path=stop_data_path,
        model_path=model_path,
        model_kind=model_kind,
        target=target,
        epochs=epochs,
        lr=lr,
        hidden_dim=hidden_dim,
        seed=seed,
        static_alpha=static_alpha,
        speed_threshold_deg_s=speed_threshold_deg_s,
        motion_history_steps=motion_history_steps,
        hz=hz,
    )


def run_cli_args(args: Any) -> Path | list[Path] | None:
    """Run a dynamics workflow from parsed CLI arguments."""
    from .main import run_cli_args as run_workflow_cli_args

    return run_workflow_cli_args(args)


__all__ = [
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
]
