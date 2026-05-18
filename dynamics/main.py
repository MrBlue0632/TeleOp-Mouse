"""Dynamics calibration command entrypoint.

Examples:
    python -m dynamics.main --mode traj --robot xarm6 --ip 192.168.1.199
    python -m dynamics.main --mode torque --traj dynamics/calibration/traj/example.parquet
    python -m dynamics.main --mode train --data dynamics/calibration/torque/example.parquet
    python -m dynamics.main --mode monitor --model-path dynamics/calibration/compensation/compensation.pt
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

from .config import build_cli_overrides, load_config


DEFAULT_TRAJ_DIR = Path("dynamics/calibration/traj")
DEFAULT_TORQUE_DIR = Path("dynamics/calibration/torque")
DEFAULT_COMPENSATION_PATH = Path("dynamics/calibration/compensation/compensation.pt")
MODES = ("traj", "torque", "train", "monitor")
TRAJ_KINDS = ("drag", "workspace", "all")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", required=True, choices=MODES)
    parser.add_argument("--robot", default="xarm6", help="robot config name")
    parser.add_argument("--config", help="YAML config path")
    parser.add_argument("--ip", help="robot IP override")
    parser.add_argument("--robot-port", type=int)
    parser.add_argument("--report-port-normal", type=int)
    parser.add_argument("--report-port-rich", type=int)
    parser.add_argument("--report-port-real", type=int)
    parser.add_argument("--urdf", help="URDF path override")
    parser.add_argument("--hz", type=float, help="sampling frequency override")
    parser.add_argument("--duration-s", type=float, help="record duration; omit to stop with Ctrl+C")
    parser.add_argument("--teach-sensitivity", type=int, help="xArm teach sensitivity 1-5")
    parser.add_argument("--traj-kind", choices=TRAJ_KINDS, help="trajectory recording submode")
    parser.add_argument("--workspace-points", type=int, help="random workspace waypoint count")
    parser.add_argument("--workspace-margin-ratio", type=float, help="workspace joint-bound shrink ratio")
    parser.add_argument("--workspace-speed-deg-s", type=float, help="generated workspace trajectory speed cap")
    parser.add_argument("--workspace-seed", type=int, help="random seed for workspace trajectory generation")
    parser.add_argument("--traj", action="append", help="trajectory parquet for torque mode; repeat for multiple files")
    parser.add_argument("--data", help="torque parquet for train mode")
    parser.add_argument("--static-data", help="optional static-pose parquet for hybrid train mode")
    parser.add_argument("--stop-data", help="optional stop-event parquet for hybrid train mode")
    parser.add_argument("--output-dir", help="output directory for traj/torque modes")
    parser.add_argument("--model-path", help="compensation model checkpoint path")
    parser.add_argument("--model-kind", choices=["baseline", "hybrid", "kinematic_history"])
    parser.add_argument("--epochs", type=int)
    parser.add_argument("--lr", type=float)
    parser.add_argument("--hidden-dim", type=int)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--static-alpha", type=float)
    parser.add_argument("--speed-threshold-deg-s", type=float)
    parser.add_argument("--motion-history-steps", type=int)
    parser.add_argument("--target", choices=["residual", "direct_api"])
    return parser.parse_args(argv)


def _section(config: dict, name: str) -> dict:
    value = config.get(name, {})
    return value if isinstance(value, dict) else {}


def _coalesce(*values):
    for value in values:
        if value is not None:
            return value
    return None


def run_mode(
    mode: str,
    config: dict,
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
    """Run one dynamics workflow mode from a loaded config."""
    paths = _section(config, "paths")
    trajectory_cfg = _section(config, "trajectory")
    torque_cfg = _section(config, "torque")
    training_cfg = _section(config, "training")
    compensation_cfg = _section(config, "compensation")
    hybrid_cfg = _section(compensation_cfg, "hybrid")
    monitor_cfg = _section(config, "monitor")

    if mode == "traj":
        from .calibration.record import record_trajectories

        return record_trajectories(
            config,
            output_dir=_coalesce(output_dir, paths.get("trajectory_dir"), DEFAULT_TRAJ_DIR),
            duration_s=_coalesce(duration_s, trajectory_cfg.get("duration_s")),
            hz=_coalesce(hz, config.get("sampling_hz")),
            teach_sensitivity=_coalesce(teach_sensitivity, trajectory_cfg.get("teach_sensitivity")),
            traj_kind=_coalesce(traj_kind, trajectory_cfg.get("kind"), "all"),
            workspace_points=int(_coalesce(workspace_points, trajectory_cfg.get("workspace_points"), 20)),
            workspace_margin_ratio=float(_coalesce(workspace_margin_ratio, trajectory_cfg.get("workspace_margin_ratio"), 0.05)),
            workspace_speed_deg_s=float(_coalesce(workspace_speed_deg_s, trajectory_cfg.get("workspace_speed_deg_s"), 30.0)),
            workspace_seed=int(_coalesce(workspace_seed, trajectory_cfg.get("workspace_seed"), 7)),
        )

    if mode == "torque":
        from .calibration.torque import collect_torque_data

        return collect_torque_data(
            config,
            traj_path=_coalesce(traj_path, torque_cfg.get("traj_path")),
            traj_dir=_coalesce(paths.get("trajectory_dir"), DEFAULT_TRAJ_DIR),
            output_dir=_coalesce(output_dir, paths.get("torque_dir"), DEFAULT_TORQUE_DIR),
            model_path=_coalesce(model_path, torque_cfg.get("model_path")),
        )

    if mode == "train":
        from .calibration.train import train_from_torque_data

        return train_from_torque_data(
            config,
            data_path=_coalesce(data_path, training_cfg.get("data_path")),
            output_path=_coalesce(model_path, paths.get("compensation_model"), DEFAULT_COMPENSATION_PATH),
            model_kind=_coalesce(model_kind, training_cfg.get("model_kind"), "baseline"),
            static_data_path=_coalesce(static_data_path, training_cfg.get("static_data_path")),
            stop_data_path=_coalesce(stop_data_path, training_cfg.get("stop_data_path")),
            target=_coalesce(target, training_cfg.get("target"), "residual"),
            epochs=int(_coalesce(epochs, training_cfg.get("epochs"), 200)),
            lr=float(_coalesce(lr, training_cfg.get("lr"), 1e-3)),
            hidden_dim=int(_coalesce(hidden_dim, training_cfg.get("hidden_dim"), 64)),
            seed=int(_coalesce(seed, training_cfg.get("seed"), 7)),
            static_alpha=float(_coalesce(static_alpha, hybrid_cfg.get("static_alpha"), 1.0)),
            speed_threshold_deg_s=float(_coalesce(speed_threshold_deg_s, hybrid_cfg.get("speed_threshold_deg_s"), 2.0)),
            motion_history_steps=int(_coalesce(motion_history_steps, hybrid_cfg.get("motion_history_steps"), 3)),
        )

    if mode == "monitor":
        from .calibration.monitor import monitor

        monitor(
            config,
            model_path=_coalesce(model_path, monitor_cfg.get("model_path")),
            hz=_coalesce(hz, config.get("sampling_hz")),
        )
        return None

    raise ValueError(f"mode must be one of {', '.join(MODES)}")


def run_cli_args(args: argparse.Namespace) -> Path | list[Path] | None:
    """Load config from CLI args and dispatch to the selected workflow mode."""
    config = load_config(
        args.config,
        robot=args.robot,
        overrides=build_cli_overrides(args),
    )
    return run_mode(
        args.mode,
        config,
        output_dir=args.output_dir,
        duration_s=args.duration_s,
        teach_sensitivity=args.teach_sensitivity,
        traj_path=args.traj,
        traj_kind=args.traj_kind,
        workspace_points=args.workspace_points,
        workspace_margin_ratio=args.workspace_margin_ratio,
        workspace_speed_deg_s=args.workspace_speed_deg_s,
        workspace_seed=args.workspace_seed,
        data_path=args.data,
        static_data_path=args.static_data,
        stop_data_path=args.stop_data,
        model_path=args.model_path,
        model_kind=args.model_kind,
        target=args.target,
        epochs=args.epochs,
        lr=args.lr,
        hidden_dim=args.hidden_dim,
        seed=getattr(args, "seed", None),
        static_alpha=getattr(args, "static_alpha", None),
        speed_threshold_deg_s=getattr(args, "speed_threshold_deg_s", None),
        motion_history_steps=getattr(args, "motion_history_steps", None),
        hz=args.hz,
    )


def main(argv: Sequence[str] | None = None) -> int:
    run_cli_args(parse_args(argv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
