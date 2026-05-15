import tempfile
import unittest

import numpy as np

from dynamics.calibration.compensation.mlp import load_compensation, save_compensation, train_compensation
from dynamics.calibration.compensation.firmware_state import FirmwareBiasTracker, FirmwareStateModel
from dynamics.calibration.compensation.hybrid import HybridTorqueCompensator, train_hybrid_compensator
from dynamics.calibration.compensation.static_bias import StaticBiasModel
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

    def test_static_bias_model_roundtrip_dict(self):
        q = np.array(
            [
                [0.0, 0.1],
                [0.2, 0.3],
                [0.4, 0.5],
                [0.6, 0.7],
            ]
        )
        residual = np.column_stack([0.5 + 0.2 * np.sin(q[:, 0]), -0.1 + 0.3 * np.cos(q[:, 1])])
        model = StaticBiasModel.fit(q, residual, alpha=0.01)
        loaded = StaticBiasModel.from_dict(model.to_dict())

        np.testing.assert_allclose(loaded.predict(q[0]), model.predict(q[0]), atol=1e-6)

    def test_firmware_tracker_updates_time_since_stop_and_levels(self):
        model = FirmwareStateModel(
            levels=[np.array([0.0]), np.array([-2.0, 2.0])],
            decay_tau_s=np.array([0.2, 0.2]),
            speed_threshold=0.1,
        )
        tracker = FirmwareBiasTracker(model)

        tracker.update(np.array([0.2, 0.2]), np.array([0.0, 0.0]), timestamp=0.0)
        est0 = tracker.update(np.array([0.0, 0.0]), np.array([0.1, 1.8]), timestamp=0.1)
        est1 = tracker.update(np.array([0.0, 0.0]), np.array([0.1, 1.5]), timestamp=0.2)

        self.assertEqual(est0.time_since_stop[1], 0.0)
        self.assertGreater(est1.time_since_stop[1], 0.0)
        self.assertEqual(est1.firmware_state[1], 2.0)

    def test_hybrid_train_save_load_predict_shape(self):
        rng = np.random.default_rng(11)
        n = 16
        j = 2
        q = rng.normal(scale=0.2, size=(n, j))
        qd = rng.normal(scale=0.03, size=(n, j))
        qdd = rng.normal(scale=0.02, size=(n, j))
        tau_theory = 0.4 * np.sin(q)
        static = np.column_stack([0.2 * np.sin(q[:, 0]), -0.1 * np.cos(q[:, 1])])
        motion = 0.05 * qd
        tau_api = tau_theory + static + motion

        bundle, metadata = train_hybrid_compensator(
            q=q,
            qd=qd,
            qdd=qdd,
            tau_api=tau_api,
            tau_model=tau_theory,
            timestamps=np.arange(n) * 0.01,
            epochs=2,
            hidden_dim=8,
        )
        self.assertEqual(metadata["joint_count"], j)
        estimate = bundle.update(q[0], qd[0], qdd[0], tau_api[0], tau_theory[0], timestamp=0.0)
        self.assertEqual(estimate.tau_external.shape, (j,))
        self.assertEqual(estimate.tau_motion_comp.shape, (j,))

        with tempfile.TemporaryDirectory() as tmp:
            path = bundle.save(f"{tmp}/hybrid.pt")
            loaded = HybridTorqueCompensator.load(path)
            estimate2 = loaded.update(q[0], qd[0], qdd[0], tau_api[0], tau_theory[0], timestamp=0.0)
        self.assertEqual(estimate2.tau_comp.shape, (j,))


if __name__ == "__main__":
    unittest.main()
