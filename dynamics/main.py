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
    parser.add_argument("--traj", help="trajectory parquet for torque mode")
    parser.add_argument("--data", help="torque parquet for train mode")
    parser.add_argument("--static-data", help="optional static-pose parquet for hybrid train mode")
    parser.add_argument("--stop-data", help="optional stop-event parquet for hybrid train mode")
    parser.add_argument("--output-dir", help="output directory for traj/torque modes")
    parser.add_argument("--model-path", help="compensation model checkpoint path")
    parser.add_argument("--model-kind", choices=["baseline", "hybrid"], default="baseline")
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--hidden-dim", type=int, default=64)
    parser.add_argument("--target", choices=["residual", "direct_api"], default="residual")
    return parser.parse_args(argv)


def run_mode(
    mode: str,
    config: dict,
    *,
    output_dir: str | Path | None = None,
    duration_s: float | None = None,
    teach_sensitivity: int | None = None,
    traj_path: str | Path | None = None,
    data_path: str | Path | None = None,
    static_data_path: str | Path | None = None,
    stop_data_path: str | Path | None = None,
    model_path: str | Path | None = None,
    model_kind: str = "baseline",
    target: str = "residual",
    epochs: int = 200,
    lr: float = 1e-3,
    hidden_dim: int = 64,
    hz: float | None = None,
) -> Path | None:
    """Run one dynamics workflow mode from a loaded config."""
    if mode == "traj":
        from .calibration.record import record_trajectory

        return record_trajectory(
            config,
            output_dir=output_dir or DEFAULT_TRAJ_DIR,
            duration_s=duration_s,
            hz=hz,
            teach_sensitivity=teach_sensitivity,
        )

    if mode == "torque":
        from .calibration.torque import collect_torque_data

        return collect_torque_data(
            config,
            traj_path=traj_path,
            output_dir=output_dir or DEFAULT_TORQUE_DIR,
            model_path=model_path,
        )

    if mode == "train":
        from .calibration.train import train_from_torque_data

        return train_from_torque_data(
            config,
            data_path=data_path,
            output_path=model_path or DEFAULT_COMPENSATION_PATH,
            model_kind=model_kind,
            static_data_path=static_data_path,
            stop_data_path=stop_data_path,
            target=target,
            epochs=epochs,
            lr=lr,
            hidden_dim=hidden_dim,
        )

    if mode == "monitor":
        from .calibration.monitor import monitor

        monitor(config, model_path=model_path, hz=hz)
        return None

    raise ValueError(f"mode must be one of {', '.join(MODES)}")


def run_cli_args(args: argparse.Namespace) -> Path | None:
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
        data_path=args.data,
        static_data_path=args.static_data,
        stop_data_path=args.stop_data,
        model_path=args.model_path,
        model_kind=args.model_kind,
        target=args.target,
        epochs=args.epochs,
        lr=args.lr,
        hidden_dim=args.hidden_dim,
        hz=args.hz,
    )


def main(argv: Sequence[str] | None = None) -> int:
    run_cli_args(parse_args(argv))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
