import argparse
import unittest
from pathlib import Path
from unittest.mock import patch

from dynamics import api
from dynamics.config import load_config
from dynamics.main import parse_args, run_cli_args


class DynamicsApiTests(unittest.TestCase):
    def test_load_config_merges_global_defaults_with_robot_config(self):
        config = load_config(robot="xarm6")

        self.assertEqual(config["robot_name"], "xarm6")
        self.assertIn("paths", config)
        self.assertEqual(config["paths"]["trajectory_dir"], "dynamics/calibration/traj")
        self.assertEqual(config["paths"]["torque_dir"], "dynamics/calibration/torque")
        self.assertEqual(config["paths"]["compensation_model"], "dynamics/calibration/compensation/compensation.pt")
        self.assertEqual(config["trajectory"]["kind"], "all")
        self.assertEqual(config["trajectory"]["workspace_points"], 20)
        self.assertEqual(config["training"]["model_kind"], "kinematic_history")
        self.assertEqual(config["training"]["epochs"], 200)
        self.assertEqual(config["compensation"]["kinematic_history"]["channels"], "q_qd")
        self.assertEqual(config["compensation"]["kinematic_history"]["window_points"], 20)
        self.assertEqual(config["compensation"]["hybrid"]["motion_history_steps"], 3)

    def test_global_config_file_is_self_contained(self):
        config = load_config("dynamics/config.yaml")

        self.assertEqual(config["robot_name"], "xarm6")
        self.assertEqual(config["joint_count"], 6)
        self.assertTrue(str(config["urdf_path"]).endswith("assets/urdf/xarm6/xarm6/xarm6.urdf"))
        self.assertEqual(config["connection"]["ip"], "192.168.1.199")
        self.assertEqual(config["payload"]["profile"], "xarm_gripper_g2")

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
            seed=None,
            static_alpha=None,
            speed_threshold_deg_s=None,
            motion_history_steps=None,
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
            seed=None,
            static_alpha=None,
            speed_threshold_deg_s=None,
            motion_history_steps=None,
            hz=100.0,
        )

    def test_parse_args_accepts_kinematic_history_model_kind(self):
        args = parse_args(["--mode", "train", "--model-kind", "kinematic_history"])

        self.assertEqual(args.model_kind, "kinematic_history")

    def test_run_mode_rejects_unknown_mode(self):
        with self.assertRaisesRegex(ValueError, "mode must be one of"):
            api.run_mode("unknown", {})

    def test_train_mode_uses_config_defaults_when_arguments_are_omitted(self):
        config = {
            "paths": {"compensation_model": "cfg-model.pt"},
            "training": {
                "model_kind": "hybrid",
                "target": "residual",
                "epochs": 9,
                "lr": 0.02,
                "hidden_dim": 33,
                "seed": 5,
            },
            "compensation": {
                "hybrid": {
                    "static_alpha": 0.25,
                    "speed_threshold_deg_s": 3.5,
                    "motion_history_steps": 2,
                }
            },
        }

        with patch("dynamics.calibration.train.train_from_torque_data", return_value=Path("cfg-model.pt")) as train:
            result = api.run_mode("train", config, data_path="data.parquet")

        self.assertEqual(result, Path("cfg-model.pt"))
        train.assert_called_once_with(
            config,
            data_path="data.parquet",
            output_path="cfg-model.pt",
            model_kind="hybrid",
            static_data_path=None,
            stop_data_path=None,
            target="residual",
            epochs=9,
            lr=0.02,
            hidden_dim=33,
            seed=5,
            static_alpha=0.25,
            speed_threshold_deg_s=3.5,
            motion_history_steps=2,
        )

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
