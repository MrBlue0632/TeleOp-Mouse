import tempfile
import unittest
from pathlib import Path

import numpy as np
import pandas as pd

from dynamics.calibration import speed_ladder as ladder
from dynamics.calibration.io import make_record, read_parquet, write_parquet


CONFIG = {
    "robot_name": "xarm6",
    "joint_count": 6,
    "home_joints_deg": [14.1, -8.0, -24.7, 196.9, 62.3, -8.8],
    "urdf_path": "assets/urdf/xarm6/xarm6/xarm6.urdf",
}


class SpeedLadderTests(unittest.TestCase):
    def test_generate_local_trajectories_count_metadata_and_home_return(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "exp"
            paths = ladder.generate_local_trajectories(
                CONFIG,
                root,
                speeds=[15.0],
                count=2,
                duration_s=2.0,
                hz=20.0,
                amplitude_deg=[2, 3, 4, 5, 6, 7],
                accel_deg_s2=45.0,
                hold_s=0.1,
                waypoint_count=2,
                seed_base=11,
                resume=False,
            )
            entries = ladder.load_trajectory_index(root)
            df = read_parquet(paths[0])

        self.assertEqual(len(paths), 2)
        self.assertEqual(len(entries), 2)
        self.assertEqual({entry.speed_tier for entry in entries}, {"15deg_s"})
        self.assertEqual(len({entry.seed for entry in entries}), 2)
        self.assertEqual(df.loc[0, "source_kind"], "local")
        self.assertEqual(df.loc[0, "speed_deg_s"], 15.0)
        q_cols = [f"q_{idx}" for idx in range(1, 7)]
        q_deg = np.rad2deg(df[q_cols].to_numpy(dtype=np.float64))
        home = np.asarray(CONFIG["home_joints_deg"], dtype=np.float64)
        np.testing.assert_allclose(q_deg[-1], home, atol=1e-6)
        self.assertTrue(np.all(np.abs(q_deg - home) <= np.asarray([2, 3, 4, 5, 6, 7]) + 1e-6))

    def test_prepare_hf_uses_action_groups_episodes_and_records_skip_reason(self):
        repo = "DorayakiLin/xarm6_pick_bread_lerobot"
        home = np.asarray(CONFIG["home_joints_deg"], dtype=np.float64)
        rows = []
        for frame in range(25):
            action = np.r_[home + frame * 0.02, 730.0].tolist()
            state = np.r_[home + 5.0, 731.0].tolist()
            rows.append(
                {
                    "episode_index": 0,
                    "frame_index": frame,
                    "timestamp": frame / 30.0,
                    "action": action,
                    "observation.state": state,
                }
            )
        for frame in range(3):
            rows.append(
                {
                    "episode_index": 1,
                    "frame_index": frame,
                    "timestamp": frame / 30.0,
                    "action": np.r_[home, 730.0].tolist(),
                    "observation.state": np.r_[home, 731.0].tolist(),
                }
            )
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "exp"
            data_dir = root / "hf_snapshots" / ladder.slug_repo(repo) / "data" / "chunk-000"
            data_dir.mkdir(parents=True)
            pd.DataFrame(rows).to_parquet(data_dir / "file-000.parquet", index=False)
            paths = ladder.prepare_hf_trajectories(
                CONFIG,
                root,
                repos=[repo],
                speeds=[15.0],
                hz=30.0,
                min_frames=20,
                max_step_deg=10.0,
                resume=False,
            )
            out = read_parquet(paths[0])
            events = ladder.load_manifest(root)

        self.assertEqual(len(paths), 1)
        self.assertEqual(out.loc[0, "source_kind"], "hf")
        self.assertEqual(out.loc[0, "source_repo"], repo)
        self.assertEqual(out.loc[0, "hf_column"], "action")
        self.assertAlmostEqual(out.loc[0, "q_1"], np.deg2rad(home[0]), places=7)
        self.assertTrue(any(event.get("reason") == "too_few_frames" for event in events))

    def test_joint_limit_filter_rejects_buffer_violations(self):
        limits = (np.array([-1.0, -1.0]), np.array([1.0, 1.0]))
        q = np.tile(np.array([[0.95, 0.0]]), (20, 1))

        ok, reason = ladder.validate_joint_trajectory(q, joint_limits=limits, min_frames=20)

        self.assertTrue(ok)
        ok, reason = ladder.validate_joint_trajectory(q, joint_limits=(np.array([-0.5, -0.5]), np.array([0.5, 0.5])), min_frames=20)
        self.assertFalse(ok)
        self.assertEqual(reason, "joint_limit_buffer_violation")

    def test_merge_splits_by_trajectory_and_validation_reports_groups(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "exp"
            for traj_idx in range(4):
                records = []
                for row_idx in range(3):
                    q = np.array([0.01 * row_idx, -0.01 * row_idx])
                    tau_model = np.array([1.0, -1.0])
                    tau_api = tau_model + np.array([0.1 * traj_idx, -0.2])
                    row = make_record(
                        timestamp=float(row_idx),
                        robot="xarm6",
                        joint_count=2,
                        mode="torque",
                        q=q,
                        qd=np.zeros(2),
                        qdd=np.zeros(2),
                        tau_api=tau_api,
                        tau_model=tau_model,
                    )
                    row.update(
                        {
                            "trajectory_id": f"traj_{traj_idx}",
                            "source_kind": "local",
                            "source_repo": "",
                            "speed_deg_s": 15.0,
                            "speed_tier": "15deg_s",
                        }
                    )
                    records.append(row)
                write_parquet(records, root / "torque" / f"traj_{traj_idx}" / "torque.parquet")
            paths = ladder.merge_torque_data(root, valid_ratio=0.5, seed=3)
            train = read_parquet(paths["train"])
            valid = read_parquet(paths["valid"])
            report = ladder.build_validation_report({"joint_count": 2}, valid, model_paths={})

        self.assertTrue(set(train["trajectory_id"]).isdisjoint(set(valid["trajectory_id"])))
        self.assertIn("model_only", report["models"])
        self.assertIn("speed_tier=15deg_s", report["models"]["model_only"])
        self.assertGreater(report["models"]["model_only"]["all"]["rows"], 0)


if __name__ == "__main__":
    unittest.main()
