"""Calibration command entrypoint.

Examples:
    python -m dynamics.calibrate --mode traj --robot xarm6 --ip 192.168.1.199
    python -m dynamics.calibrate --mode torque --traj dynamics/calibration/traj/example.parquet
    python -m dynamics.calibrate --mode train --data dynamics/calibration/torque/example.parquet
    python -m dynamics.calibrate --mode monitor --model-path dynamics/calibration/compensation/compensation.pt
"""

from __future__ import annotations

import argparse

from .calibration.monitor import monitor
from .calibration.record import record_trajectory
from .calibration.torque import collect_torque_data
from .calibration.train import train_from_torque_data
from .config import build_cli_overrides, load_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", required=True, choices=["traj", "torque", "train", "monitor"])
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
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_config(args.config, robot=args.robot, overrides=build_cli_overrides(args))

    if args.mode == "traj":
        record_trajectory(
            config,
            output_dir=args.output_dir or "dynamics/calibration/traj",
            duration_s=args.duration_s,
            hz=args.hz,
            teach_sensitivity=args.teach_sensitivity,
        )
    elif args.mode == "torque":
        collect_torque_data(
            config,
            traj_path=args.traj,
            output_dir=args.output_dir or "dynamics/calibration/torque",
            model_path=args.model_path,
        )
    elif args.mode == "train":
        train_from_torque_data(
            config,
            data_path=args.data,
            output_path=args.model_path or "dynamics/calibration/compensation/compensation.pt",
            model_kind=args.model_kind,
            static_data_path=args.static_data,
            stop_data_path=args.stop_data,
            target=args.target,
            epochs=args.epochs,
            lr=args.lr,
            hidden_dim=args.hidden_dim,
        )
    elif args.mode == "monitor":
        monitor(config, model_path=args.model_path, hz=args.hz)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
