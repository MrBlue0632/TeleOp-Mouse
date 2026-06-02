"""Torque estimation helpers shared by record teleop and diagnostics.

Inputs: xArm joint position, speed, API torque, and optional compensation model.
Returns: model, compensation, firmware-bias, and estimated external torque arrays.
"""

import os
import time

import numpy as np

from .constants import N_JOINTS, TORQUE_DEAD_ZONE


MOTION_SPEED_THR = 2.0
BLEND_ALPHA = 0.2
SETTLE_LAM_THR = 0.05
DETECT_EMA_ALPHA = 0.05
ADMITTANCE_FILTER_ALPHA = 0.3
COMPENSATION_MAX_ABS_NM = 25.0
STOP_SETTLE_S = 1.0
STOP_SPIKE_NORM_NM = 10.0
OBSERVER_BLEND_MOTION = 0.65
OBSERVER_BLEND_STATIC = 0.15
OBSERVER_BLEND_SETTLING = 0.75

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


def _as_joint_vector(values):
    return np.asarray(values, dtype=np.float64)[:N_JOINTS].copy()


def _apply_admittance_dead_zone(tau_ext):
    """Apply the same per-joint dead-zone used by Xarm_force admittance."""
    out = _as_joint_vector(tau_ext)
    for i in range(N_JOINTS):
        if abs(out[i]) < TORQUE_DEAD_ZONE[i]:
            out[i] = 0.0
        else:
            out[i] -= np.sign(out[i]) * TORQUE_DEAD_ZONE[i]
    return out


class FirmwareBiasTorqueEstimator:
    """Estimate admittance-style external joint torque from xArm API torques.

    Inputs are joint position in degrees, joint speed in deg/s, and measured
    joint torque in Nm. The public ``tau_external`` output follows the
    Xarm_force admittance convention: model/compensation torque minus measured
    API torque, then dead-zone and EMA filtering.
    """

    def __init__(
        self,
        dynamics=None,
        compensation_path: str | None = None,
        compensation_sample_hz: float | None = None,
        filter_alpha: float = ADMITTANCE_FILTER_ALPHA,
    ):
        if dynamics is None:
            from .dynamics import DynamicsModel

            dynamics = DynamicsModel()
        self.dynamics = dynamics
        self.compensator = None
        self.hybrid = None  # Backward-compatible alias for older callers/tests.
        self.hybrid_error = None
        self.compensation_path = None
        self.compensation_kind = "none"
        self.compensation_ready = False
        self.compensation_sample_hz = compensation_sample_hz
        self.compensation_max_abs_nm = float(os.getenv("TELEOP_TORQUE_COMP_MAX_ABS_NM", COMPENSATION_MAX_ABS_NM))
        self.filter_alpha = float(filter_alpha)
        self.last_compensation_reject = None
        self.tau_external_filt = np.zeros(N_JOINTS, dtype=np.float64)
        self.stop_settle_s = float(os.getenv("TELEOP_TORQUE_STOP_SETTLE_S", STOP_SETTLE_S))
        self.stop_spike_norm_nm = float(os.getenv("TELEOP_TORQUE_STOP_SPIKE_NORM_NM", STOP_SPIKE_NORM_NM))
        self.observer = None
        self.observer_error = None
        self._last_update_t: float | None = None
        self._last_motion_t: float | None = None
        self._last_stop_t: float | None = None
        self._was_moving = False
        self._stop_event_id = 0
        self.stop_phase = "static"
        self.lam = 0.0
        self.detected_bias = np.zeros(N_JOINTS, dtype=np.float64)
        self.resid_ema = np.zeros(N_JOINTS, dtype=np.float64)
        self.ema_initialized = False
        self._last_qd_rad: np.ndarray | None = None
        self._last_t: float | None = None

        self._configure_observer()

        model_path = compensation_path or os.getenv("TELEOP_TORQUE_COMP_MODEL")
        if model_path:
            expanded = os.path.abspath(os.path.expanduser(model_path))
            self.compensation_path = expanded
            if not os.path.exists(expanded):
                self.hybrid_error = f"compensation file not found: {expanded}"
            else:
                try:
                    from dynamics.calibration.compensation import load_compensation

                    comp = load_compensation(expanded)
                    if not (hasattr(comp, "update") or hasattr(comp, "predict_compensation")):
                        raise TypeError(f"unsupported compensation type: {type(comp).__name__}")
                    self.compensator = comp
                    self.hybrid = comp
                    self.compensation_kind = str(getattr(comp, "kind", type(comp).__name__))
                    self._configure_compensation_sample_hz()
                except Exception as exc:
                    self.hybrid_error = str(exc)
                    self.compensator = None
                    self.hybrid = None
                    self.compensation_kind = "error"

    def _configure_observer(self):
        enabled = os.getenv("TELEOP_TORQUE_USE_OBSERVER", "1").strip().lower() not in {"0", "false", "no", "off"}
        if not enabled or not hasattr(self.dynamics, "_mass_matrix_rad"):
            return
        try:
            from .dynamics import MomentumObserver

            gain = float(os.getenv("TELEOP_TORQUE_OBSERVER_GAIN", "50.0"))
            self.observer = MomentumObserver(self.dynamics, gain=gain)
        except Exception as exc:
            self.observer_error = str(exc)
            self.observer = None

    def _configure_compensation_sample_hz(self):
        if self.compensator is None or self.compensation_sample_hz is None:
            return
        if not hasattr(self.compensator, "control_hz"):
            return
        try:
            self.compensator.control_hz = float(self.compensation_sample_hz)
            post_init = getattr(self.compensator, "__post_init__", None)
            if callable(post_init):
                post_init()
        except Exception as exc:
            self.hybrid_error = f"failed to set compensation sample hz: {exc}"

    def _update_motion_phase(self, now, is_moving_raw):
        if is_moving_raw:
            self._was_moving = True
            self._last_motion_t = now
            self.stop_phase = "motion"
            return 0.0
        if self._was_moving:
            self._was_moving = False
            self._last_stop_t = now
            self._stop_event_id += 1
        if self._last_stop_t is None:
            self.stop_phase = "static"
            return 0.0
        time_since_stop = max(0.0, float(now - self._last_stop_t))
        self.stop_phase = "settling" if time_since_stop <= self.stop_settle_s else "static"
        return time_since_stop

    def _update_firmware_bias(self, qd, tau, tau_model, now):
        max_speed = float(np.abs(qd).max()) if qd.size else 0.0
        is_moving_raw = bool(max_speed > MOTION_SPEED_THR)
        time_since_stop = self._update_motion_phase(now, is_moving_raw)
        is_moving = float(is_moving_raw)
        self.lam = (1.0 - BLEND_ALPHA) * self.lam + BLEND_ALPHA * is_moving

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

        return self.detected_bias * (1.0 - self.lam), time_since_stop

    def _kinematics_rad(self, q, qd):
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
        return now, q_rad, qd_rad, qdd_rad

    def _validate_compensation(self, tau_comp):
        arr = _as_joint_vector(tau_comp)
        if not np.all(np.isfinite(arr)):
            return False, "non-finite compensation output"
        max_abs = float(np.max(np.abs(arr))) if arr.size else 0.0
        if max_abs > self.compensation_max_abs_nm:
            return False, f"compensation output too large: {max_abs:.3f} Nm > {self.compensation_max_abs_nm:.3f} Nm"
        return True, None

    def _observer_direct_residual(self, q, qd, tau, now):
        zeros = np.zeros(N_JOINTS, dtype=np.float64)
        if self.observer is None:
            return zeros, False
        if self._last_update_t is None:
            dt = 1.0 / float(self.compensation_sample_hz or 100.0)
        else:
            dt = max(float(now - self._last_update_t), 1e-4)
        try:
            direct = _as_joint_vector(self.observer.update(q, qd, tau, dt))
            if not np.all(np.isfinite(direct)):
                return zeros, False
            return direct, True
        except Exception as exc:
            self.observer_error = str(exc)
            self.observer = None
            return zeros, False

    def _observer_blend(self):
        if self.stop_phase == "motion":
            return OBSERVER_BLEND_MOTION
        if self.stop_phase == "settling":
            return OBSERVER_BLEND_SETTLING
        return OBSERVER_BLEND_STATIC

    def _compensation_estimate(self, q, qd, tau, tau_model, tau_jump):
        zeros = np.zeros(N_JOINTS, dtype=np.float64)
        tau_comp = zeros.copy()
        tau_static = zeros.copy()
        tau_motion = zeros.copy()
        tau_firmware = tau_jump.copy()
        time_since_stop = zeros.copy()
        firmware_state = self.detected_bias.copy()
        comp_ready = False
        comp_used = False
        comp_rejected = False
        lam = float(self.lam)
        self.last_compensation_reject = None

        if self.compensator is None:
            direct_residual = tau - tau_model - tau_firmware
            return direct_residual, tau_comp, tau_static, tau_motion, tau_firmware, time_since_stop, firmware_state, comp_used, comp_ready, comp_rejected, lam

        now, q_rad, qd_rad, qdd_rad = self._kinematics_rad(q, qd)
        if hasattr(self.compensator, "update"):
            estimate = self.compensator.update(q_rad, qd_rad, qdd_rad, tau, tau_model, timestamp=now)
            comp_ready = bool(getattr(estimate, "is_ready", True))
            if comp_ready:
                tau_comp = _as_joint_vector(getattr(estimate, "tau_comp", zeros))
                ok, reason = self._validate_compensation(tau_comp)
                if ok:
                    tau_static = _as_joint_vector(getattr(estimate, "tau_static_bias", zeros))
                    tau_motion = _as_joint_vector(getattr(estimate, "tau_motion_comp", zeros))
                    tau_firmware = _as_joint_vector(getattr(estimate, "tau_firmware_bias", zeros))
                    time_since_stop = _as_joint_vector(getattr(estimate, "time_since_stop", zeros))
                    firmware_state = _as_joint_vector(getattr(estimate, "firmware_state", zeros))
                    motion_lambda = getattr(estimate, "motion_lambda", None)
                    lam = float(self.lam if motion_lambda is None else motion_lambda)
                    direct_residual = _as_joint_vector(getattr(estimate, "tau_external", tau - tau_model - tau_comp))
                    comp_used = True
                else:
                    self.last_compensation_reject = reason
                    comp_rejected = True
                    tau_comp = zeros.copy()
                    direct_residual = tau - tau_model - tau_firmware
            else:
                direct_residual = tau - tau_model - tau_firmware
        else:
            tau_comp = _as_joint_vector(self.compensator.predict_compensation(q_rad, qd_rad, qdd_rad, tau_model))
            ok, reason = self._validate_compensation(tau_comp)
            comp_ready = True
            if ok:
                tau_motion = tau_comp.copy()
                direct_residual = tau - tau_model - tau_comp
                comp_used = True
            else:
                self.last_compensation_reject = reason
                comp_rejected = True
                tau_comp = zeros.copy()
                direct_residual = tau - tau_model - tau_firmware

        return direct_residual, tau_comp, tau_static, tau_motion, tau_firmware, time_since_stop, firmware_state, comp_used, comp_ready, comp_rejected, lam

    def update(self, q_deg, qd_dps, tau_api):
        q = _as_joint_vector(q_deg)
        qd = _as_joint_vector(qd_dps)
        tau = _as_joint_vector(tau_api)
        now = time.time()

        tau_model = self.dynamics.gravity(q) + self.dynamics.coriolis(q, qd)
        tau_jump, stop_time_scalar = self._update_firmware_bias(qd, tau, tau_model, now)
        observer_direct, observer_ok = self._observer_direct_residual(q, qd, tau, now)
        (
            direct_residual,
            tau_comp,
            tau_static,
            tau_motion,
            tau_firmware,
            time_since_stop,
            firmware_state,
            comp_used,
            comp_ready,
            comp_rejected,
            lam,
        ) = self._compensation_estimate(q, qd, tau, tau_model, tau_jump)

        fallback_time_since_stop = np.full(N_JOINTS, stop_time_scalar, dtype=np.float64)
        if time_since_stop is None or not np.any(time_since_stop):
            time_since_stop = fallback_time_since_stop

        # Xarm_force admittance convention: tau_ext = tau_model + compensation - tau_meas.
        tau_external_direct = -_as_joint_vector(direct_residual)
        tau_external_observer = -observer_direct if observer_ok else np.zeros(N_JOINTS, dtype=np.float64)
        if observer_ok:
            blend = self._observer_blend()
            tau_external_raw = (1.0 - blend) * tau_external_direct + blend * tau_external_observer
        else:
            tau_external_raw = tau_external_direct.copy()

        direct_norm = float(np.linalg.norm(tau_external_direct))
        confidence = 1.0
        valid_no_contact = True
        if self.stop_phase == "settling":
            confidence = 0.45
            if direct_norm > self.stop_spike_norm_nm:
                confidence = 0.15
                valid_no_contact = False
                tau_external_raw = tau_external_raw * confidence
        if comp_rejected:
            confidence = min(confidence, 0.5)
            valid_no_contact = False

        tau_external_deadzone = _apply_admittance_dead_zone(tau_external_raw)
        alpha = min(1.0, max(0.0, self.filter_alpha))
        self.tau_external_filt = alpha * tau_external_deadzone + (1.0 - alpha) * self.tau_external_filt
        self.compensation_ready = bool(comp_ready)
        self._last_update_t = now

        return {
            "lambda": float(lam),
            "tau_api": tau,
            "tau_model": tau_model,
            "tau_comp": tau_comp,
            "tau_static_bias": tau_static,
            "tau_motion_comp": tau_motion,
            "tau_firmware_bias": tau_firmware,
            "tau_external_direct": tau_external_direct,
            "tau_external_observer": tau_external_observer,
            "tau_external_raw": tau_external_raw,
            "tau_external_deadzone": tau_external_deadzone,
            "tau_external": self.tau_external_filt.copy(),
            "time_since_stop": time_since_stop,
            "firmware_state": firmware_state,
            "stop_phase": self.stop_phase,
            "stop_event_id": int(self._stop_event_id),
            "estimator_confidence": float(confidence),
            "valid_no_contact": bool(valid_no_contact),
            "observer_enabled": observer_ok,
            "observer_error": self.observer_error,
            "compensation_enabled": self.compensator is not None,
            "compensation_used": bool(comp_used),
            "compensation_ready": bool(comp_ready),
            "compensation_rejected": bool(comp_rejected),
            "compensation_kind": self.compensation_kind,
            "compensation_path": self.compensation_path,
            "compensation_error": self.last_compensation_reject or self.hybrid_error,
        }

    def close(self):
        close = getattr(self.dynamics, "close", None)
        if callable(close):
            close()
