"""Real-time rigid-body dynamics for xArm6 via PyBullet inverse dynamics.

Provides gravity, Coriolis, mass-matrix, and full inverse-dynamics torque
computation, plus a **Momentum Observer** (De Luca 2005) that estimates
external joint torques without differentiating q̇.

Typical per-call latency: ~0.013 ms (capable of >70 kHz).
"""

import numpy as np
import pybullet as pb

from .constants import N_JOINTS, PB_JOINT_INDICES, XARM6_URDF

_EPS_GRAD = 1e-4  # step size for numerical gradient of M(q)


class DynamicsModel:
    """xArm6 inverse dynamics model.

    All public methods accept angles in **degrees** and angular velocities
    in **deg/s**.  Returned torques are in **Nm**.
    """

    def __init__(self, urdf_path: str = XARM6_URDF):
        self._cid = pb.connect(pb.DIRECT)
        pb.setGravity(0, 0, -9.81, physicsClientId=self._cid)
        self._robot = pb.loadURDF(
            urdf_path, useFixedBase=True, physicsClientId=self._cid,
        )
        for j in PB_JOINT_INDICES:
            pb.changeDynamics(
                self._robot, j,
                linearDamping=0, angularDamping=0,
                physicsClientId=self._cid,
            )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _set_q(self, q_rad):
        for i, j in enumerate(PB_JOINT_INDICES):
            pb.resetJointState(
                self._robot, j, q_rad[i], physicsClientId=self._cid,
            )

    def _id(self, q_rad, qd_rad, qdd_rad) -> np.ndarray:
        """Raw inverse dynamics call (radians in, Nm out)."""
        self._set_q(q_rad)
        tau = pb.calculateInverseDynamics(
            self._robot,
            list(q_rad), list(qd_rad), list(qdd_rad),
            physicsClientId=self._cid,
        )
        return np.array(tau[:N_JOINTS])

    def _mass_matrix_rad(self, q_rad) -> np.ndarray:
        """M(q) in radians, returns (N, N)."""
        self._set_q(q_rad)
        M = pb.calculateMassMatrix(self._robot, list(q_rad),
                                   physicsClientId=self._cid)
        return np.array(M)

    def _mdot_qd(self, q_rad, qd_rad) -> np.ndarray:
        """Ṁ(q,q̇)·q̇ via central-difference gradient of M(q).

        Ṁ = Σ_k (∂M/∂q_k)·q̇_k,  so  Ṁ·q̇ = Σ_k q̇_k · (∂M/∂q_k)·q̇
        """
        qd = np.asarray(qd_rad)
        result = np.zeros(N_JOINTS)
        for k in range(N_JOINTS):
            if abs(qd[k]) < 1e-10:
                continue
            q_plus = q_rad.copy()
            q_minus = q_rad.copy()
            q_plus[k] += _EPS_GRAD
            q_minus[k] -= _EPS_GRAD
            dM_dqk = (self._mass_matrix_rad(q_plus)
                       - self._mass_matrix_rad(q_minus)) / (2 * _EPS_GRAD)
            result += qd[k] * (dM_dqk @ qd)
        return result

    # ------------------------------------------------------------------
    # Public API (degrees / deg/s in, Nm out)
    # ------------------------------------------------------------------

    def gravity(self, q_deg: np.ndarray) -> np.ndarray:
        """G(q) — gravity torque vector."""
        q = np.deg2rad(q_deg)
        z = [0.0] * N_JOINTS
        return self._id(q, z, z)

    def coriolis(self, q_deg: np.ndarray, qd_dps: np.ndarray) -> np.ndarray:
        """C(q, q̇)·q̇ — Coriolis + centrifugal torque vector."""
        q = np.deg2rad(q_deg)
        qd = np.deg2rad(qd_dps)
        z = [0.0] * N_JOINTS
        return self._id(q, qd, z) - self._id(q, z, z)

    def mass_matrix(self, q_deg: np.ndarray) -> np.ndarray:
        """M(q) — joint-space inertia matrix (N×N)."""
        return self._mass_matrix_rad(np.deg2rad(q_deg))

    def inverse_dynamics(
        self,
        q_deg: np.ndarray,
        qd_dps: np.ndarray,
        qdd_dps2: np.ndarray,
    ) -> np.ndarray:
        """M(q)·q̈ + C(q,q̇)·q̇ + G(q) — full inverse dynamics."""
        return self._id(
            np.deg2rad(q_deg),
            np.deg2rad(qd_dps),
            np.deg2rad(qdd_dps2),
        )

    def eta(self, q_deg: np.ndarray, qd_dps: np.ndarray) -> np.ndarray:
        """η(q,q̇) ≈ C·q̇ + G  (used by momentum observer).

        Drops the Ṁ·q̇ correction — it scales as O(q̇²) and is negligible
        at typical operating speeds (<30 °/s → <0.1 Nm).  Skipping it
        eliminates 2N mass-matrix evaluations per call (~6× speedup for η).
        """
        q = np.deg2rad(q_deg)
        qd = np.deg2rad(qd_dps)
        z = [0.0] * N_JOINTS
        return self._id(q, qd, z)       # C·q̇ + G

    def close(self):
        if pb.isConnected(self._cid):
            pb.disconnect(self._cid)


class MomentumObserver:
    """Generalized-momentum observer for external torque estimation.

    Uses integration (not differentiation) to estimate τ_ext, giving
    fast response without noise amplification.

    Reference: De Luca & Mattone, IEEE ICRA 2005.

    Parameters
    ----------
    dyn : DynamicsModel
    gain : float
        Observer gain K_O (1/s).  Higher = faster convergence.
        Time constant ≈ 1/gain.  Default 50 → ~20 ms response.
    bias_model : GravityBiasModel | None
        Position-dependent bias model. If provided, bias(q) is subtracted
        from the observer output instead of a constant bias.
    """

    # Median filter on tau_meas → observer input (noise + spike rejection).
    _MEDIAN_W = 5

    # Stationary bias update from raw signals (no τ_ext feedback).
    # Two-speed: fast after sudden change (firmware jump), slow otherwise.
    # Conservative: URDF model is accurate, bias only handles residual drift.
    _BIAS_ALPHA_FAST = 0.05  # after sudden jump → absorb in ~300ms
    _BIAS_ALPHA_SLOW = 0.005 # baseline drift → minimal force absorption
    _BIAS_SPEED_THR = 1.0    # deg/s — only adapt when stationary
    _BIAS_ERR_THR = 5.0      # Nm — max error to absorb (above = likely real force)
    _BIAS_SUDDEN_THR = 1.5   # Nm — raw_error rate to trigger fast mode
    _BIAS_FAST_STEPS = 15    # steps to stay in fast mode after trigger

    def __init__(self, dyn: DynamicsModel, gain: float = 50.0,
                 bias_model=None):
        self._dyn = dyn
        self._gain = gain
        self._sigma: np.ndarray | None = None
        self._bias = np.zeros(N_JOINTS)
        self._bias_model = bias_model
        # Median filter buffer for observer input
        self._tau_buf = np.zeros((self._MEDIAN_W, N_JOINTS))
        self._buf_idx = 0
        self._buf_count = 0
        # Two-speed bias state
        self._raw_error_prev = np.zeros(N_JOINTS)
        self._fast_count = np.zeros(N_JOINTS, dtype=int)

    def update(
        self,
        q_deg: np.ndarray,
        qd_dps: np.ndarray,
        tau_meas: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """Run one observer step.  Returns τ_ext estimate (Nm)."""
        # --- 1. Median filter ---
        self._tau_buf[self._buf_idx] = tau_meas
        self._buf_idx = (self._buf_idx + 1) % self._MEDIAN_W
        self._buf_count = min(self._buf_count + 1, self._MEDIAN_W)
        tau_filt = np.median(self._tau_buf[:self._buf_count], axis=0)

        # --- 2. Observer ---
        q_rad = np.deg2rad(q_deg)
        qd_rad = np.deg2rad(qd_dps)
        M = self._dyn._mass_matrix_rad(q_rad)
        p = M @ qd_rad
        eta = self._dyn.eta(q_deg, qd_dps)

        if self._sigma is None:
            self._sigma = p.copy()
            return np.zeros(N_JOINTS)

        r = self._gain * (self._sigma - p)
        self._sigma += (tau_filt - eta - r) * dt

        if self._bias_model is not None:
            r = r - self._bias_model.predict(q_deg)
        else:
            r = r - self._bias

        # --- 3. Stationary bias: fast after sudden jump, slow otherwise ---
        if np.abs(qd_dps).max() < self._BIAS_SPEED_THR:
            grav = self._dyn.gravity(q_deg)
            raw_error = tau_meas - grav - self._bias
            for i in range(N_JOINTS):
                if abs(raw_error[i]) >= self._BIAS_ERR_THR:
                    continue
                # Detect sudden change → firmware jump → fast mode
                delta_err = abs(raw_error[i] - self._raw_error_prev[i])
                if delta_err > self._BIAS_SUDDEN_THR:
                    self._fast_count[i] = self._BIAS_FAST_STEPS
                # Apply appropriate alpha
                if self._fast_count[i] > 0:
                    self._fast_count[i] -= 1
                    self._bias[i] += self._BIAS_ALPHA_FAST * raw_error[i]
                else:
                    self._bias[i] += self._BIAS_ALPHA_SLOW * raw_error[i]
            self._raw_error_prev[:] = raw_error

        return r

    def reset_tracking(self):
        """No-op, kept for API compatibility."""
        pass

    def calibrate(self, samples: list[np.ndarray]):
        """Set constant bias from calibration samples (fallback when no model)."""
        self._bias = np.mean(samples, axis=0)
        print(f"[CAL] Bias set: {['%.3f' % b for b in self._bias]}")
