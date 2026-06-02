import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from dynamics.backends.base import RobotSample
from dynamics.calibration.io import make_record, read_parquet, write_parquet
from dynamics.calibration.torque import collect_torque_data, resolve_traj_paths


class TorqueTrajectoryResolutionTests(unittest.TestCase):
    def write_traj(self, directory: str, filename: str) -> Path:
        return write_parquet(
            [
                make_record(
                    timestamp=1.0,
                    robot="xarm6",
                    joint_count=2,
                    mode="traj",
                    q=np.array([0.1, 0.2]),
                    qd=np.array([0.0, 0.0]),
                    qdd=np.array([0.0, 0.0]),
                )
            ],
            Path(directory) / filename,
        )

    def test_explicit_multiple_trajectory_paths_are_preserved(self):
        with tempfile.TemporaryDirectory() as tmp:
            drag = self.write_traj(tmp, "drag.parquet")
            workspace = self.write_traj(tmp, "workspace.parquet")

            paths = resolve_traj_paths([str(drag), str(workspace)], tmp)

        self.assertEqual(paths, [drag, workspace])

    def test_default_resolution_returns_latest_drag_and_workspace_pair(self):
        with tempfile.TemporaryDirectory() as tmp:
            old_drag = self.write_traj(tmp, "xarm6_traj_drag_20260101_000000.parquet")
            drag = self.write_traj(tmp, "xarm6_traj_drag_20260101_000001.parquet")
            workspace = self.write_traj(tmp, "xarm6_traj_workspace_20260101_000002.parquet")
            old_drag.touch()
            drag.touch()
            workspace.touch()

            paths = resolve_traj_paths(None, tmp)

        self.assertEqual(paths, [drag, workspace])

    def test_collect_torque_data_applies_replay_speed_and_metadata(self):
        class FakeBackend:
            def __init__(self):
                self.replay_kwargs = None
                self.read_count = 0

            def connect(self):
                return None

            def close(self):
                return None

            def replay_joint_positions(self, **kwargs):
                self.replay_kwargs = kwargs

            def read_sample(self):
                self.read_count += 1
                return RobotSample(
                    timestamp=float(self.read_count),
                    q=np.array([0.1, 0.2]),
                    qd=np.array([0.0, 0.0]),
                    tau_api=np.array([1.0, 2.0]),
                    raw={},
                )

        class FakeTheoreticalModel:
            def __init__(self, _robot):
                return None

            def __enter__(self):
                return object()

            def __exit__(self, *_args):
                return False

        fake_backend = FakeBackend()
        estimate = SimpleNamespace(
            tau_api=np.array([1.0, 2.0]),
            tau_theory=np.array([0.5, 1.0]),
            tau_comp=np.zeros(2),
            tau_error=np.array([0.5, 1.0]),
            tau_static_bias=np.zeros(2),
            tau_motion_comp=np.zeros(2),
            tau_firmware_bias=np.zeros(2),
            tau_external=np.array([0.5, 1.0]),
            time_since_stop=np.zeros(2),
            firmware_state=np.zeros(2),
            motion_lambda=0.0,
            is_moving=False,
        )

        with tempfile.TemporaryDirectory() as tmp:
            traj = self.write_traj(tmp, "traj.parquet")
            with patch("dynamics.calibration.torque.create_backend", return_value=fake_backend), patch(
                "dynamics.calibration.torque.resolve_robot", return_value=object()
            ), patch("dynamics.calibration.torque.TheoreticalModel", FakeTheoreticalModel), patch(
                "dynamics.calibration.torque.estimate_torque_sample", return_value=estimate
            ):
                out = collect_torque_data(
                    {"robot_name": "xarm6", "joint_count": 2, "urdf_path": "robot.urdf"},
                    traj_path=traj,
                    output_dir=Path(tmp) / "torque",
                    replay_speed_deg_s=25.0,
                    replay_acc_deg_s2=300.0,
                    metadata={
                        "experiment_id": "exp1",
                        "trajectory_id": "traj1",
                        "speed_deg_s": 25.0,
                        "speed_tier": "25deg_s",
                        "source_kind": "local",
                        "source_repo": "",
                        "episode_index": -1,
                        "seed": 5,
                        "ignored_list": [1, 2],
                    },
                )
            df = read_parquet(out)

        self.assertEqual(fake_backend.replay_kwargs["speed_deg_s"], 25.0)
        self.assertEqual(fake_backend.replay_kwargs["acc_deg_s2"], 300.0)
        self.assertEqual(df.loc[0, "experiment_id"], "exp1")
        self.assertEqual(df.loc[0, "trajectory_id"], "traj1")
        self.assertEqual(df.loc[0, "speed_tier"], "25deg_s")
        self.assertEqual(df.loc[0, "replay_speed_deg_s"], 25.0)
        self.assertNotIn("ignored_list", df.columns)


if __name__ == "__main__":
    unittest.main()
