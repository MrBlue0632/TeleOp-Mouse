import numpy as np

from record.control.torque_estimation import FirmwareBiasTorqueEstimator


class DummyDynamics:
    def gravity(self, _q):
        return np.full(6, 10.0, dtype=np.float64)

    def coriolis(self, _q, _qd):
        return np.zeros(6, dtype=np.float64)


def test_firmware_bias_estimator_records_admittance_external_torque():
    estimator = FirmwareBiasTorqueEstimator(dynamics=DummyDynamics())
    q = np.zeros(6, dtype=np.float64)
    qd = np.full(6, 3.0, dtype=np.float64)
    tau_api = np.array([8.0, 7.0, 8.0, 8.0, 9.0, 9.0], dtype=np.float64)

    estimate = estimator.update(q, qd, tau_api)

    np.testing.assert_allclose(
        estimate["tau_external_raw"],
        [2.0, 3.0, 2.0, 2.0, 1.0, 1.0],
    )
    np.testing.assert_allclose(
        estimate["tau_external_deadzone"],
        [0.5, 1.5, 1.0, 1.2, 0.5, 0.7],
    )
    np.testing.assert_allclose(
        estimate["tau_external"],
        [0.15, 0.45, 0.3, 0.36, 0.15, 0.21],
    )


def test_kinematic_history_compensation_checkpoint_loads(tmp_path):
    pytest = __import__("pytest")
    pytest.importorskip("torch")
    from dynamics.calibration.compensation import KinematicHistoryCompensator

    model_path = tmp_path / "history.pt"
    comp = KinematicHistoryCompensator.zeros(6, control_hz=10.0, window_points=1)
    comp.save(model_path)

    estimator = FirmwareBiasTorqueEstimator(
        dynamics=DummyDynamics(),
        compensation_path=str(model_path),
        compensation_sample_hz=10.0,
        filter_alpha=1.0,
    )
    estimate = estimator.update(np.zeros(6), np.zeros(6), np.full(6, 10.0))

    assert estimator.compensation_kind == "kinematic_history"
    assert estimate["compensation_enabled"] is True
    assert estimate["compensation_used"] is True
    assert estimate["compensation_ready"] is True
    np.testing.assert_allclose(estimate["tau_external"], np.zeros(6), atol=1e-6)


class _HugeCompensationEstimate:
    is_ready = True
    motion_lambda = 0.0
    time_since_stop = np.zeros(6, dtype=np.float64)
    firmware_state = np.zeros(6, dtype=np.float64)
    tau_static_bias = np.zeros(6, dtype=np.float64)
    tau_firmware_bias = np.zeros(6, dtype=np.float64)
    tau_motion_comp = np.full(6, 100000.0, dtype=np.float64)
    tau_comp = tau_motion_comp.copy()
    tau_external = np.full(6, -100000.0, dtype=np.float64)


class _HugeCompensator:
    kind = "huge"

    def update(self, *_args, **_kwargs):
        return _HugeCompensationEstimate()


def test_rejects_unbounded_compensation_output():
    estimator = FirmwareBiasTorqueEstimator(dynamics=DummyDynamics(), filter_alpha=1.0)
    estimator.compensator = _HugeCompensator()
    estimator.compensation_kind = "huge"

    estimate = estimator.update(
        np.zeros(6, dtype=np.float64),
        np.full(6, 3.0, dtype=np.float64),
        np.array([8.0, 7.0, 8.0, 8.0, 9.0, 9.0], dtype=np.float64),
    )

    assert estimate["compensation_enabled"] is True
    assert estimate["compensation_used"] is False
    assert estimate["compensation_rejected"] is True
    assert "too large" in estimate["compensation_error"]
    np.testing.assert_allclose(estimate["tau_motion_comp"], np.zeros(6))
    np.testing.assert_allclose(estimate["tau_external"], [0.5, 1.5, 1.0, 1.2, 0.5, 0.7])


def test_stop_transition_spike_is_low_confidence_and_damped():
    estimator = FirmwareBiasTorqueEstimator(dynamics=DummyDynamics(), filter_alpha=1.0)
    q = np.zeros(6, dtype=np.float64)

    moving = estimator.update(q, np.full(6, 3.0, dtype=np.float64), np.full(6, 10.0, dtype=np.float64))
    assert moving["stop_phase"] == "motion"

    spike_tau = np.array([10.0, 35.0, 30.0, 10.0, 10.0, 10.0], dtype=np.float64)
    estimate = estimator.update(q, np.zeros(6, dtype=np.float64), spike_tau)

    assert estimate["stop_phase"] == "settling"
    assert estimate["estimator_confidence"] < 0.5
    assert estimate["valid_no_contact"] is False
    assert np.linalg.norm(estimate["tau_external_direct"]) > 10.0
    assert np.linalg.norm(estimate["tau_external"]) < np.linalg.norm(estimate["tau_external_direct"])
    assert estimate["stop_event_id"] == 1
