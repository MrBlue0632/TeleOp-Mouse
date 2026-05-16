import unittest
from types import SimpleNamespace

import numpy as np

from dynamics.backends.base import RobotSample
from dynamics.calibration.runtime import estimate_torque_sample


class FakeModel:
    def estimate_joint_torque(self, q, qd, qdd):
        return np.asarray(q, dtype=np.float64) + np.asarray(qd, dtype=np.float64) + np.asarray(qdd, dtype=np.float64)


class BaselineCompensator:
    def predict_compensation(self, q, qd, qdd, tau_theory=None):
        self.tau_theory = tau_theory
        return np.array([0.1, 0.2])


class HybridCompensator:
    def update(self, q, qd, qdd, tau_api, tau_theory, timestamp=None):
        self.timestamp = timestamp
        return SimpleNamespace(
            tau_comp=np.array([0.3, 0.4]),
            tau_static_bias=np.array([0.1, 0.1]),
            tau_motion_comp=np.array([0.2, 0.2]),
            tau_firmware_bias=np.array([0.0, 0.1]),
            tau_external=np.array([1.0, 1.1]),
            time_since_stop=np.array([0.0, 0.2]),
            firmware_state=np.array([0.0, 2.0]),
            motion_lambda=0.75,
            is_moving=np.array([False, True]),
        )


class RuntimeTorqueEstimateTests(unittest.TestCase):
    def make_sample(self):
        return RobotSample(
            timestamp=12.5,
            q=np.array([1.0, 2.0]),
            qd=np.array([0.2, 0.3]),
            tau_api=np.array([2.0, 3.0]),
        )

    def test_estimates_sample_without_compensator(self):
        estimate = estimate_torque_sample(
            sample=self.make_sample(),
            qd=np.array([0.2, 0.3]),
            qdd=np.array([0.01, 0.02]),
            joint_count=2,
            model=FakeModel(),
        )

        np.testing.assert_allclose(estimate.tau_theory, [1.21, 2.32])
        np.testing.assert_allclose(estimate.tau_comp, [0.0, 0.0])
        np.testing.assert_allclose(estimate.tau_error, [0.79, 0.68])
        np.testing.assert_allclose(estimate.tau_external, estimate.tau_error)

    def test_estimates_sample_with_baseline_compensator(self):
        compensator = BaselineCompensator()
        estimate = estimate_torque_sample(
            sample=self.make_sample(),
            qd=np.array([0.2, 0.3]),
            qdd=np.array([0.01, 0.02]),
            joint_count=2,
            model=FakeModel(),
            compensator=compensator,
        )

        np.testing.assert_allclose(compensator.tau_theory, [1.21, 2.32])
        np.testing.assert_allclose(estimate.tau_comp, [0.1, 0.2])
        np.testing.assert_allclose(estimate.tau_error, [0.69, 0.48])
        np.testing.assert_allclose(estimate.tau_external, estimate.tau_error)

    def test_estimates_sample_with_stateful_hybrid_compensator(self):
        compensator = HybridCompensator()
        estimate = estimate_torque_sample(
            sample=self.make_sample(),
            qd=np.array([0.2, 0.3]),
            qdd=np.array([0.01, 0.02]),
            joint_count=2,
            model=FakeModel(),
            compensator=compensator,
        )

        self.assertEqual(compensator.timestamp, 12.5)
        self.assertTrue(estimate.is_moving)
        self.assertEqual(estimate.motion_lambda, 0.75)
        np.testing.assert_allclose(estimate.tau_comp, [0.3, 0.4])
        np.testing.assert_allclose(estimate.tau_external, [1.0, 1.1])
        np.testing.assert_allclose(estimate.time_since_stop, [0.0, 0.2])
        np.testing.assert_allclose(estimate.firmware_state, [0.0, 2.0])


if __name__ == "__main__":
    unittest.main()
