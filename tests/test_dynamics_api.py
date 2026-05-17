import argparse
import unittest
from pathlib import Path
from unittest.mock import patch

from dynamics import api
from dynamics.main import parse_args, run_cli_args


class DynamicsApiTests(unittest.TestCase):
    def test_parse_args_accepts_explicit_argv(self):
        args = parse_args(
            [
                "--mode",
                "traj",
                "--robot",
                "xarm6",
                "--hz",
                "50",
                "--traj-kind",
                "workspace",
                "--traj",
                "drag.parquet",
                "--traj",
                "workspace.parquet",
                "--workspace-points",
                "8",
                "--workspace-speed-deg-s",
                "25",
                "--workspace-seed",
                "11",
            ]
        )

        self.assertEqual(args.mode, "traj")
        self.assertEqual(args.robot, "xarm6")
        self.assertEqual(args.hz, 50.0)
        self.assertEqual(args.traj_kind, "workspace")
        self.assertEqual(args.traj, ["drag.parquet", "workspace.parquet"])
        self.assertEqual(args.workspace_points, 8)
        self.assertEqual(args.workspace_speed_deg_s, 25.0)
        self.assertEqual(args.workspace_seed, 11)

    def test_run_cli_args_loads_config_and_dispatches_mode(self):
        args = argparse.Namespace(
            mode="torque",
            robot="xarm6",
            config=None,
            ip="192.168.1.10",
            robot_port=None,
            report_port_normal=None,
            report_port_rich=None,
            report_port_real=None,
            urdf=None,
            hz=100.0,
            output_dir="out",
            duration_s=None,
            teach_sensitivity=None,
            traj=["traj.parquet", "workspace.parquet"],
            traj_kind="all",
            workspace_points=20,
            workspace_margin_ratio=0.05,
            workspace_speed_deg_s=30.0,
            workspace_seed=7,
            data=None,
            static_data=None,
            stop_data=None,
            model_path="model.pt",
            model_kind="baseline",
            target="residual",
            epochs=10,
            lr=0.01,
            hidden_dim=16,
        )
        config = {"robot_name": "xarm6"}

        with patch("dynamics.main.load_config", return_value=config) as load_config:
            with patch("dynamics.main.run_mode", return_value=Path("out.parquet")) as run_mode:
                result = run_cli_args(args)

        self.assertEqual(result, Path("out.parquet"))
        load_config.assert_called_once()
        run_mode.assert_called_once_with(
            "torque",
            config,
            output_dir="out",
            duration_s=None,
            teach_sensitivity=None,
            traj_path=["traj.parquet", "workspace.parquet"],
            traj_kind="all",
            workspace_points=20,
            workspace_margin_ratio=0.05,
            workspace_speed_deg_s=30.0,
            workspace_seed=7,
            data_path=None,
            static_data_path=None,
            stop_data_path=None,
            model_path="model.pt",
            model_kind="baseline",
            target="residual",
            epochs=10,
            lr=0.01,
            hidden_dim=16,
            hz=100.0,
        )

    def test_run_mode_rejects_unknown_mode(self):
        with self.assertRaisesRegex(ValueError, "mode must be one of"):
            api.run_mode("unknown", {})

    def test_api_delegates_workflow_calls_to_main(self):
        args = argparse.Namespace(mode="monitor")

        with patch("dynamics.main.run_mode", return_value=Path("mode.out")) as run_mode:
            mode_result = api.run_mode("traj", {"robot_name": "xarm6"}, duration_s=1.0)
        with patch("dynamics.main.run_cli_args", return_value=Path("cli.out")) as run_cli_args:
            cli_result = api.run_cli_args(args)

        self.assertEqual(mode_result, Path("mode.out"))
        self.assertEqual(cli_result, Path("cli.out"))
        run_mode.assert_called_once()
        run_cli_args.assert_called_once_with(args)


if __name__ == "__main__":
    unittest.main()
