#!/usr/bin/env python3
"""Reset a supported robot arm to its configured home position."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from dynamics.backends import create_backend
from dynamics.config import build_cli_overrides, load_config


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="xarm6", help="robot config name, default: xarm6")
    parser.add_argument("--config", help="YAML config path")
    parser.add_argument("--ip", help="robot IP; overrides config")
    parser.add_argument("--robot-port", type=int, help="xArm control port; overrides config")
    parser.add_argument("--report-port-normal", type=int, help="xArm normal report port")
    parser.add_argument("--report-port-rich", type=int, help="xArm rich report port")
    parser.add_argument("--report-port-real", type=int, help="xArm real-time report port")
    parser.add_argument("--urdf", help="URDF path; parsed by shared config but not needed for reset")
    parser.add_argument("--no-gripper", action="store_true", help="do not move gripper")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config = load_config(args.config, robot=args.robot, overrides=build_cli_overrides(args))
    home = config.get("home_joints_deg")
    if not home:
        raise SystemExit("config is missing home_joints_deg")
    gripper_open = None if args.no_gripper else config.get("gripper_open")
    backend = create_backend(config, robot=args.robot)
    try:
        backend.connect()
        backend.reset_home(home, gripper_open=gripper_open)
        print(f"[OK] {config.get('robot_name', args.robot)} reset to home")
    finally:
        backend.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
