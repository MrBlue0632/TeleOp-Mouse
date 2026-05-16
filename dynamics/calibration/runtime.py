"""Shared runtime torque-estimation helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from dynamics.backends.base import RobotSample


@dataclass
class TorqueSampleEstimate:
    tau_api: np.ndarray
    tau_theory: np.ndarray
    tau_comp: np.ndarray
    tau_error: np.ndarray
    tau_static_bias: np.ndarray
    tau_motion_comp: np.ndarray
    tau_firmware_bias: np.ndarray
    tau_external: np.ndarray
    time_since_stop: np.ndarray | None = None
    firmware_state: np.ndarray | None = None
    motion_lambda: float | None = None
    is_moving: bool | None = None


def estimate_torque_sample(
    *,
    sample: RobotSample,
    qd: np.ndarray,
    qdd: np.ndarray,
    joint_count: int,
    model: Any,
    compensator: Any | None = None,
) -> TorqueSampleEstimate:
    """Estimate theory, compensation, and residual torque for one robot sample."""
    tau_api = sample.tau_api if sample.tau_api is not None else np.zeros(joint_count)
    tau_theory = model.estimate_joint_torque(sample.q, qd, qdd)
    zeros = np.zeros(joint_count)
    tau_static_bias = zeros.copy()
    tau_motion_comp = zeros.copy()
    tau_firmware_bias = zeros.copy()
    tau_external = None
    time_since_stop = None
    firmware_state = None
    motion_lambda = None
    is_moving = None

    if compensator is None:
        tau_comp = zeros.copy()
    elif hasattr(compensator, "update"):
        estimate = compensator.update(sample.q, qd, qdd, tau_api, tau_theory, timestamp=sample.timestamp)
        tau_comp = estimate.tau_comp
        tau_static_bias = estimate.tau_static_bias
        tau_motion_comp = estimate.tau_motion_comp
        tau_firmware_bias = estimate.tau_firmware_bias
        tau_external = estimate.tau_external
        time_since_stop = estimate.time_since_stop
        firmware_state = estimate.firmware_state
        motion_lambda = estimate.motion_lambda
        is_moving = bool(np.any(estimate.is_moving))
    else:
        tau_comp = compensator.predict_compensation(sample.q, qd, qdd, tau_theory=tau_theory)

    tau_error = tau_api - tau_theory - tau_comp
    if tau_external is None:
        tau_external = tau_error

    return TorqueSampleEstimate(
        tau_api=tau_api,
        tau_theory=tau_theory,
        tau_comp=tau_comp,
        tau_error=tau_error,
        tau_static_bias=tau_static_bias,
        tau_motion_comp=tau_motion_comp,
        tau_firmware_bias=tau_firmware_bias,
        tau_external=tau_external,
        time_since_stop=time_since_stop,
        firmware_state=firmware_state,
        motion_lambda=motion_lambda,
        is_moving=is_moving,
    )
