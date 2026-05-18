"""Torque estimation helpers shared by record teleop and diagnostics.

Inputs: xArm joint position, speed, API torque, and optional compensation model.
Returns: model, compensation, firmware-bias, and estimated external torque arrays.
"""

import os
import time

import numpy as np

from .constants import N_JOINTS


MOTION_SPEED_THR = 2.0
BLEND_ALPHA = 0.2
SETTLE_LAM_THR = 0.05
DETECT_EMA_ALPHA = 0.05

BIAS_LEVELS = {
    1: (0.0000, 0.0000),
    2: (-4.5056, 4.0151),
    3: (-2.4267, 3.3561),
    4: (0.0857, 0.0857),
    5: (-1.9928, 0.9617),
    6: (0.1845, 0.1845),
}

BIAS_MIDPOINTS = np.array(
    [(BIAS_LEVELS[j + 1][0] + BIAS_LEVELS[j + 1][1]) / 2.0 for j in range(N_JOINTS)],
    dtype=np.float64,
)
BIAS_IS_BIMODAL = np.array(
    [BIAS_LEVELS[j + 1][0] != BIAS_LEVELS[j + 1][1] for j in range(N_JOINTS)],
    dtype=bool,
)


class FirmwareBiasTorqueEstimator:
    """Estimate external joint torque from xArm API torque reports.

    Inputs are joint position in degrees, joint speed in deg/s, and measured
    joint torque in Nm. Outputs are Nm.
    """

    def __init__(self, dynamics=None, compensation_path: str | None = None):
        if dynamics is None:
            from .dynamics import DynamicsModel

            dynamics = DynamicsModel()
        self.dynamics = dynamics
        self.hybrid = None
        self.hybrid_error = None
        self.lam = 0.0
        self.detected_bias = np.zeros(N_JOINTS, dtype=np.float64)
        self.resid_ema = np.zeros(N_JOINTS, dtype=np.float64)
        self.ema_initialized = False
        self._last_qd_rad: np.ndarray | None = None
        self._last_t: float | None = None
        model_path = compensation_path or os.getenv("TELEOP_TORQUE_COMP_MODEL")
        if model_path and os.path.exists(os.path.expanduser(model_path)):
            try:
                from dynamics.calibration.compensation import HybridTorqueCompensator, load_compensation

                comp = load_compensation(os.path.expanduser(model_path))
                if isinstance(comp, HybridTorqueCompensator):
                    self.hybrid = comp
            except Exception as exc:
                self.hybrid_error = str(exc)

    def update(self, q_deg, qd_dps, tau_api):
        q = np.asarray(q_deg, dtype=np.float64)[:N_JOINTS]
        qd = np.asarray(qd_dps, dtype=np.float64)[:N_JOINTS]
        tau = np.asarray(tau_api, dtype=np.float64)[:N_JOINTS]

        max_speed = float(np.abs(qd).max()) if qd.size else 0.0
        is_moving = float(max_speed > MOTION_SPEED_THR)
        self.lam = (1.0 - BLEND_ALPHA) * self.lam + BLEND_ALPHA * is_moving

        tau_model = self.dynamics.gravity(q) + self.dynamics.coriolis(q, qd)
        if self.hybrid is not None:
            now = time.time()
            q_rad = np.deg2rad(q)
            qd_rad = np.deg2rad(qd)
            if self._last_qd_rad is None or self._last_t is None:
                qdd_rad = np.zeros(N_JOINTS, dtype=np.float64)
            else:
                dt = max(now - self._last_t, 1e-6)
                qdd_rad = (qd_rad - self._last_qd_rad) / dt
            self._last_qd_rad = qd_rad.copy()
            self._last_t = now
            estimate = self.hybrid.update(q_rad, qd_rad, qdd_rad, tau, tau_model, timestamp=now)
            return {
                "lambda": float(estimate.motion_lambda),
                "tau_api": tau,
                "tau_model": tau_model,
                "tau_comp": estimate.tau_comp,
                "tau_static_bias": estimate.tau_static_bias,
                "tau_motion_comp": estimate.tau_motion_comp,
                "tau_firmware_bias": estimate.tau_firmware_bias,
                "tau_external": estimate.tau_external,
                "time_since_stop": estimate.time_since_stop,
                "firmware_state": estimate.firmware_state,
            }

        resid = tau - tau_model

        if self.lam < SETTLE_LAM_THR:
            if not self.ema_initialized:
                self.resid_ema[:] = resid
                self.ema_initialized = True
            else:
                self.resid_ema += DETECT_EMA_ALPHA * (resid - self.resid_ema)

            for j in range(N_JOINTS):
                if BIAS_IS_BIMODAL[j]:
                    lo, hi = BIAS_LEVELS[j + 1]
                    self.detected_bias[j] = hi if self.resid_ema[j] > BIAS_MIDPOINTS[j] else lo
                else:
                    self.detected_bias[j] = BIAS_LEVELS[j + 1][0]
        else:
            self.ema_initialized = False

        tau_comp = np.zeros(N_JOINTS, dtype=np.float64)
        tau_jump = self.detected_bias * (1.0 - self.lam)
        tau_external = tau - tau_model - tau_comp - tau_jump

        return {
            "lambda": float(self.lam),
            "tau_api": tau,
            "tau_model": tau_model,
            "tau_comp": tau_comp,
            "tau_static_bias": np.zeros(N_JOINTS, dtype=np.float64),
            "tau_motion_comp": tau_comp,
            "tau_firmware_bias": tau_jump,
            "tau_external": tau_external,
            "time_since_stop": np.zeros(N_JOINTS, dtype=np.float64),
            "firmware_state": self.detected_bias.copy(),
        }

    def close(self):
        close = getattr(self.dynamics, "close", None)
        if callable(close):
            close()
