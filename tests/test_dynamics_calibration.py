import tempfile
import unittest

import numpy as np

from dynamics.calibration.compensation.mlp import load_compensation, save_compensation, train_compensation
from dynamics.calibration.io import LowLatencyDifferentiator, extract_matrix, make_record, read_parquet, write_parquet


class CalibrationIoTests(unittest.TestCase):
    def test_parquet_roundtrip_for_vector_columns(self):
        records = [
            make_record(
                timestamp=1.0,
                robot="xarm6",
                joint_count=2,
                mode="traj",
                q=np.array([0.1, 0.2]),
                qd=np.array([0.3, 0.4]),
                qdd=np.array([0.5, 0.6]),
            )
        ]
        with tempfile.TemporaryDirectory() as tmp:
            path = write_parquet(records, f"{tmp}/traj.parquet")
            df = read_parquet(path)

        np.testing.assert_allclose(extract_matrix(df, "q", 2), [[0.1, 0.2]])
        np.testing.assert_allclose(extract_matrix(df, "qd", 2), [[0.3, 0.4]])
        self.assertEqual(df.loc[0, "robot"], "xarm6")

    def test_low_latency_differentiator_uses_velocity_input(self):
        diff = LowLatencyDifferentiator(1, alpha=1.0)
        qd0, qdd0 = diff.update(0.0, np.array([0.0]), np.array([0.0]))
        qd1, qdd1 = diff.update(0.01, np.array([0.0]), np.array([2.0]))

        np.testing.assert_allclose(qd0, [0.0])
        np.testing.assert_allclose(qdd0, [0.0])
        np.testing.assert_allclose(qd1, [2.0])
        np.testing.assert_allclose(qdd1, [200.0])


class CompensationTests(unittest.TestCase):
    def test_train_save_load_predict_shape(self):
        rng = np.random.default_rng(7)
        n = 12
        j = 2
        q = rng.normal(size=(n, j))
        qd = rng.normal(size=(n, j))
        qdd = rng.normal(size=(n, j))
        tau_theory = rng.normal(size=(n, j))
        tau_api = tau_theory + 0.1 * q + 0.2 * qd

        bundle, losses = train_compensation(q, qd, qdd, tau_api, tau_theory, epochs=3, hidden_dim=8)
        self.assertEqual(len(losses), 3)
        pred = bundle.predict_compensation(q[0], qd[0], qdd[0], tau_theory=tau_theory[0])
        self.assertEqual(pred.shape, (j,))

        with tempfile.TemporaryDirectory() as tmp:
            path = save_compensation(bundle, f"{tmp}/model.pt")
            loaded = load_compensation(path)
            pred2 = loaded.predict_compensation(q[0], qd[0], qdd[0], tau_theory=tau_theory[0])
        self.assertEqual(pred2.shape, (j,))


if __name__ == "__main__":
    unittest.main()
