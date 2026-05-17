import tempfile
import unittest
from pathlib import Path

import numpy as np

from dynamics.calibration.compensation.mlp import load_compensation, save_compensation, train_compensation
from dynamics.calibration.compensation.firmware_state import FirmwareBiasTracker, FirmwareStateModel, KinematicFirmwareBiasTracker
from dynamics.calibration.compensation.hybrid import HybridTorqueCompensator, train_hybrid_compensator
from dynamics.calibration.compensation.static_bias import StaticBiasModel
from dynamics.calibration.io import LowLatencyDifferentiator, extract_matrix, make_record, read_parquet, write_parquet
from dynamics.calibration.train import train_from_torque_data
from dynamics.calibration.workspace import estimate_workspace_bounds, generate_random_workspace_trajectory
from dynamics.config import load_config


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
                traj_kind="drag",
            )
        ]
        with tempfile.TemporaryDirectory() as tmp:
            path = write_parquet(records, f"{tmp}/traj.parquet")
            df = read_parquet(path)

        np.testing.assert_allclose(extract_matrix(df, "q", 2), [[0.1, 0.2]])
        np.testing.assert_allclose(extract_matrix(df, "qd", 2), [[0.3, 0.4]])
        self.assertEqual(df.loc[0, "robot"], "xarm6")
        self.assertEqual(df.loc[0, "traj_kind"], "drag")

    def test_low_latency_differentiator_uses_velocity_input(self):
        diff = LowLatencyDifferentiator(1, alpha=1.0)
        qd0, qdd0 = diff.update(0.0, np.array([0.0]), np.array([0.0]))
        qd1, qdd1 = diff.update(0.01, np.array([0.0]), np.array([2.0]))

        np.testing.assert_allclose(qd0, [0.0])
        np.testing.assert_allclose(qdd0, [0.0])
        np.testing.assert_allclose(qd1, [2.0])
        np.testing.assert_allclose(qdd1, [200.0])

    def test_workspace_bounds_apply_margin_per_joint(self):
        samples = np.array(
            [
                [0.0, 1.0],
                [10.0, 21.0],
                [4.0, 9.0],
            ]
        )

        bounds = estimate_workspace_bounds(samples, margin_ratio=0.1)

        np.testing.assert_allclose(bounds.lower, [1.0, 3.0])
        np.testing.assert_allclose(bounds.upper, [9.0, 19.0])

    def test_random_workspace_trajectory_stays_in_bounds_and_speed_limit(self):
        samples = np.array(
            [
                [-0.5, 0.0],
                [0.5, 1.0],
            ]
        )
        bounds = estimate_workspace_bounds(samples, margin_ratio=0.0)

        q_traj, timestamps = generate_random_workspace_trajectory(
            bounds,
            point_count=5,
            hz=20.0,
            speed_deg_s=30.0,
            seed=3,
            start_q=np.array([0.0, 0.5]),
        )

        self.assertEqual(q_traj.shape[1], 2)
        self.assertEqual(q_traj.shape[0], timestamps.shape[0])
        self.assertGreater(q_traj.shape[0], 5)
        self.assertTrue(np.all(q_traj >= bounds.lower - 1e-12))
        self.assertTrue(np.all(q_traj <= bounds.upper + 1e-12))
        dt = np.diff(timestamps)
        joint_speed = np.abs(np.diff(q_traj, axis=0)) / dt[:, None]
        self.assertLessEqual(float(np.max(joint_speed)), np.deg2rad(30.0) + 1e-9)


class CompensationTests(unittest.TestCase):
    def make_torque_records(self, joint_count: int = 6, rows: int = 8):
        records = []
        for idx in range(rows):
            q = np.full(joint_count, 0.01 * idx, dtype=np.float64)
            qd = np.full(joint_count, 0.02, dtype=np.float64)
            qdd = np.full(joint_count, 0.001, dtype=np.float64)
            tau_model = np.linspace(0.1, 0.6, joint_count) + 0.01 * idx
            tau_api = tau_model + np.linspace(0.01, 0.06, joint_count)
            records.append(
                make_record(
                    timestamp=float(idx) * 0.01,
                    robot="xarm6",
                    joint_count=joint_count,
                    mode="torque",
                    q=q,
                    qd=qd,
                    qdd=qdd,
                    tau_api=tau_api,
                    tau_model=tau_model,
                )
            )
        return records

    def test_embodiment_metadata_includes_robot_identity(self):
        from dynamics.embodiment import build_embodiment_metadata

        config = load_config(robot="xarm6")
        metadata = build_embodiment_metadata(config)

        self.assertEqual(metadata["id"], "xarm6_g2_augmented_urdf")
        self.assertEqual(metadata["robot_name"], "xarm6")
        self.assertEqual(metadata["joint_count"], 6)
        self.assertEqual(metadata["joint_names"], ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"])
        self.assertEqual(len(metadata["urdf_sha256"]), 64)
        self.assertEqual(metadata["units"]["q"], "rad")
        self.assertEqual(metadata["payload"]["profile"], "xarm_gripper_g2")
        self.assertEqual(metadata["payload"]["mode"], "augment_urdf")
        self.assertTrue(metadata["payload"]["apply_to_model"])

    def test_train_from_torque_data_writes_baseline_embodiment_metadata(self):
        import torch

        config = load_config(robot="xarm6")
        with tempfile.TemporaryDirectory() as tmp:
            data_path = write_parquet(self.make_torque_records(), Path(tmp) / "torque.parquet")
            out = train_from_torque_data(
                config,
                data_path=data_path,
                output_path=Path(tmp) / "baseline.pt",
                epochs=1,
                hidden_dim=4,
            )
            checkpoint = torch.load(out, map_location="cpu", weights_only=False)

        embodiment = checkpoint["extra"]["embodiment"]
        self.assertEqual(embodiment["id"], "xarm6_g2_augmented_urdf")
        self.assertEqual(embodiment["joint_count"], 6)

    def test_config_aware_loader_rejects_mismatched_hybrid_embodiment(self):
        import torch

        from dynamics.embodiment import build_embodiment_metadata
        from dynamics.calibration.compensation import load_compensation_for_config

        rng = np.random.default_rng(13)
        n = 8
        j = 6
        q = rng.normal(scale=0.1, size=(n, j))
        qd = rng.normal(scale=0.02, size=(n, j))
        qdd = rng.normal(scale=0.01, size=(n, j))
        tau_model = 0.2 * np.sin(q)
        tau_api = tau_model + 0.03 * qd
        config = load_config(robot="xarm6")
        bundle, _ = train_hybrid_compensator(
            q=q,
            qd=qd,
            qdd=qdd,
            tau_api=tau_api,
            tau_model=tau_model,
            timestamps=np.arange(n) * 0.01,
            epochs=1,
            hidden_dim=4,
            embodiment=build_embodiment_metadata(config),
        )

        with tempfile.TemporaryDirectory() as tmp:
            path = bundle.save(Path(tmp) / "hybrid.pt")
            loaded = load_compensation_for_config(path, config)
            self.assertEqual(loaded.metadata["embodiment"]["id"], "xarm6_g2_augmented_urdf")

            checkpoint = torch.load(path, map_location="cpu", weights_only=False)
            checkpoint["metadata"]["embodiment"]["urdf_sha256"] = "0" * 64
            torch.save(checkpoint, path)

            with self.assertRaisesRegex(ValueError, "embodiment mismatch.*urdf_sha256"):
                load_compensation_for_config(path, config)

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

    def test_motion_history_features_use_previous_kinematic_window(self):
        from dynamics.calibration.compensation.per_joint_mlp import build_motion_history_features

        q = np.array([[1.0, 2.0], [3.0, 4.0], [5.0, 6.0]])
        qd = q + 10.0
        qdd = q + 20.0

        history = build_motion_history_features(q, qd, qdd, history_steps=2)

        self.assertEqual(history.shape, (3, 12))
        np.testing.assert_allclose(history[0], np.zeros(12))
        np.testing.assert_allclose(history[1], [1.0, 2.0, 11.0, 12.0, 21.0, 22.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        np.testing.assert_allclose(history[2], [3.0, 4.0, 13.0, 14.0, 23.0, 24.0, 1.0, 2.0, 11.0, 12.0, 21.0, 22.0])

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

    def test_kinematic_jump_tracker_uses_directional_level_and_delay_kernel(self):
        model = FirmwareStateModel(
            levels=[np.array([-2.0, 2.0])],
            decay_tau_s=np.array([0.1]),
            direction_levels=np.array([[-2.0, 0.0, 2.0]]),
            delay_s=np.array([0.05]),
            speed_threshold=0.1,
        )
        tracker = KinematicFirmwareBiasTracker(model)

        tracker.update(np.array([0.0]), np.array([0.2]), np.array([0.0]), timestamp=0.0)
        stopped = tracker.update(np.array([0.0]), np.array([0.0]), np.array([0.0]), timestamp=0.1)
        delayed = tracker.update(np.array([0.0]), np.array([0.0]), np.array([0.0]), timestamp=0.14)
        rising = tracker.update(np.array([0.0]), np.array([0.0]), np.array([0.0]), timestamp=0.25)

        np.testing.assert_allclose(stopped.bias, [0.0])
        np.testing.assert_allclose(delayed.bias, [0.0])
        self.assertGreater(rising.bias[0], 1.0)
        self.assertLess(rising.bias[0], 2.0)
        np.testing.assert_allclose(rising.firmware_state, [2.0])

    def test_derived_time_since_stop_matches_runtime_transition_semantics(self):
        from dynamics.calibration.compensation.firmware_state import derive_time_since_stop

        qd = np.array([[0.2], [0.0], [0.0], [0.2], [0.0], [0.0]])
        timestamps = np.arange(qd.shape[0], dtype=np.float64) * 0.1

        time_since_stop = derive_time_since_stop(qd, timestamps, speed_threshold=0.1)

        np.testing.assert_allclose(time_since_stop[:, 0], [0.0, 0.0, 0.1, 0.0, 0.0, 0.1])

    def test_hybrid_runtime_prediction_ignores_tau_api_feedback(self):
        class ZeroMotionModel:
            def predict(self, q, qd, qdd, tau_model, motion_lambda, time_since_stop):
                return np.zeros(2, dtype=np.float64)

        comp = HybridTorqueCompensator(
            static_model=StaticBiasModel.zeros(2),
            firmware_model=FirmwareStateModel(
                levels=[np.array([0.0]), np.array([-2.0, 2.0])],
                decay_tau_s=np.array([0.2, 0.2]),
                speed_threshold=0.1,
            ),
            motion_model=ZeroMotionModel(),
        )
        q = np.array([0.1, -0.2])
        qdd = np.zeros(2)
        tau_model = np.array([0.3, -0.4])

        def run_with(tau_api):
            comp.reset()
            comp.update(q, np.array([0.2, 0.2]), qdd, tau_api, tau_model, timestamp=0.0)
            comp.update(q, np.zeros(2), qdd, tau_api, tau_model, timestamp=0.1)
            return comp.update(q, np.zeros(2), qdd, tau_api, tau_model, timestamp=0.2)

        low = run_with(np.array([1.0, 1.0]))
        high = run_with(np.array([11.0, 11.0]))

        np.testing.assert_allclose(low.tau_comp, high.tau_comp)
        np.testing.assert_allclose(high.tau_external - low.tau_external, [10.0, 10.0])

    def test_hybrid_runtime_passes_kinematic_history_to_motion_model(self):
        class CapturingMotionModel:
            history_steps = 1

            def __init__(self):
                self.history_features = []

            def predict(self, q, qd, qdd, tau_model, motion_lambda, time_since_stop, history_features=None):
                self.history_features.append(None if history_features is None else np.asarray(history_features).copy())
                return np.zeros(2, dtype=np.float64)

        motion_model = CapturingMotionModel()
        comp = HybridTorqueCompensator(
            static_model=StaticBiasModel.zeros(2),
            firmware_model=FirmwareStateModel.defaults(2, speed_threshold=0.1),
            motion_model=motion_model,
        )

        comp.update(
            np.array([0.1, 0.2]),
            np.array([0.3, 0.4]),
            np.array([0.5, 0.6]),
            np.zeros(2),
            np.zeros(2),
            timestamp=0.0,
        )
        comp.update(
            np.array([1.1, 1.2]),
            np.array([1.3, 1.4]),
            np.array([1.5, 1.6]),
            np.zeros(2),
            np.zeros(2),
            timestamp=0.1,
        )

        np.testing.assert_allclose(motion_model.history_features[0], [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])
        np.testing.assert_allclose(motion_model.history_features[1], [[0.1, 0.2, 0.3, 0.4, 0.5, 0.6]])

    def test_train_from_torque_data_accepts_tau_model_columns(self):
        records = []
        for idx in range(8):
            q = np.array([0.01 * idx, -0.02 * idx])
            qd = np.array([0.03, -0.04])
            qdd = np.array([0.001, -0.002])
            tau_model = np.array([0.2 + 0.01 * idx, -0.1])
            tau_api = tau_model + np.array([0.05, -0.02])
            row = make_record(
                timestamp=float(idx) * 0.01,
                robot="xarm6",
                joint_count=2,
                mode="torque",
                q=q,
                qd=qd,
                qdd=qdd,
                tau_api=tau_api,
            )
            row["tau_model_1"] = float(tau_model[0])
            row["tau_model_2"] = float(tau_model[1])
            records.append(row)

        with tempfile.TemporaryDirectory() as tmp:
            data_path = write_parquet(records, f"{tmp}/torque.parquet")
            out = train_from_torque_data(
                {"joint_count": 2},
                data_path=data_path,
                output_path=f"{tmp}/model.pt",
                epochs=1,
                hidden_dim=4,
            )

        self.assertEqual(out.name, "model.pt")

    def test_validation_reports_component_and_stop_window_rmse(self):
        from dynamics.calibration.validation import evaluate_torque_breakdown

        timestamps = np.array([0.0, 0.1, 0.2, 0.3])
        qd = np.array([[0.2], [0.0], [0.0], [0.0]])
        tau_model = np.array([[1.0], [1.0], [1.0], [1.0]])
        tau_static = np.array([[0.2], [0.2], [0.2], [0.2]])
        tau_motion = np.array([[0.1], [0.0], [0.0], [0.0]])
        tau_jump = np.array([[0.0], [0.0], [0.5], [0.8]])
        tau_api = tau_model + tau_static + tau_motion + tau_jump

        metrics = evaluate_torque_breakdown(
            qd=qd,
            tau_api=tau_api,
            tau_model=tau_model,
            tau_static=tau_static,
            tau_motion=tau_motion,
            tau_jump_delay=tau_jump,
            timestamps=timestamps,
            speed_threshold=0.1,
        )

        self.assertGreater(metrics["model_only"]["rmse"], metrics["full"]["rmse"])
        self.assertEqual(metrics["full"]["rmse"], 0.0)
        self.assertEqual(metrics["full"]["stop_0_1s_rmse"], 0.0)
        self.assertIn("per_joint_rmse", metrics["static_motion_jump"])

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
        self.assertEqual(metadata["target"], "tau_api_minus_tau_model")
        self.assertIn("delayed_jump", metadata["components"])
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
