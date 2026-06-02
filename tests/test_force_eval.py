import tempfile
import unittest
from pathlib import Path

import numpy as np

from dynamics.calibration.force_eval import derive_motion_masks, evaluate_force_estimation_path
from dynamics.calibration.io import make_record, write_parquet


class ForceEvalTests(unittest.TestCase):
    def test_evaluates_torque_parquet_and_stop_window(self):
        records = []
        q = np.zeros(2, dtype=np.float64)
        for idx, (qd, tau_ext) in enumerate(
            [
                ([0.1, 0.0], [1.0, 2.0]),
                ([0.1, 0.0], [1.0, 2.0]),
                ([0.0, 0.0], [4.0, 8.0]),
                ([0.0, 0.0], [1.0, 1.0]),
            ]
        ):
            records.append(
                make_record(
                    timestamp=float(idx) * 0.1,
                    robot="xarm6",
                    joint_count=2,
                    mode="torque",
                    q=q,
                    qd=np.asarray(qd, dtype=np.float64),
                    qdd=np.zeros(2, dtype=np.float64),
                    tau_external=np.asarray(tau_ext, dtype=np.float64),
                )
            )
        with tempfile.TemporaryDirectory() as tmp:
            path = write_parquet(records, Path(tmp) / "torque.parquet")
            result = evaluate_force_estimation_path(path, joint_count=2, speed_threshold_deg_s=2.0)

        self.assertEqual(result["rows"], 4)
        self.assertEqual(result["metrics"]["motion"]["frames"], 2)
        self.assertEqual(result["metrics"]["static"]["frames"], 2)
        self.assertGreaterEqual(result["metrics"]["stop_0_1s"]["frames"], 1)
        self.assertGreater(result["metrics"]["all"]["norm_max_nm"], 8.0)

    def test_derive_motion_masks_prefers_action_for_lerobot_frames(self):
        import pandas as pd

        df = pd.DataFrame(
            {
                "timestamp": [0.0, 0.1, 0.2],
                "action": [np.zeros(7), np.array([1, 0, 0, 0, 0, 0, 0]), np.zeros(7)],
                "observation.torque_external": [np.zeros(6), np.zeros(6), np.zeros(6)],
            }
        )
        masks = derive_motion_masks(df)

        self.assertFalse(bool(masks["motion"][0]))
        self.assertTrue(bool(masks["motion"][1]))
        self.assertTrue(bool(masks["stop_0_1s"][2]))


if __name__ == "__main__":
    unittest.main()
