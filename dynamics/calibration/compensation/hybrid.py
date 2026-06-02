"""Hybrid torque compensation model.

The hybrid model decomposes xArm torque API residuals into three parts:
static position bias, motion-related residuals, and firmware/current state
bias after motion stops.
"""

from __future__ import annotations

from dataclasses import dataclass
import inspect
from pathlib import Path
from typing import Any

import numpy as np
import torch

from .firmware_state import (
    FirmwareStateModel,
    KinematicFirmwareBiasTracker,
    derive_motion_lambda,
    derive_time_since_stop,
)
from .per_joint_mlp import PerJointMotionModel, train_per_joint_motion_model
from .static_bias import StaticBiasModel


@dataclass
class HybridTorqueEstimate:
    tau_external: np.ndarray
    tau_static_bias: np.ndarray
    tau_motion_comp: np.ndarray
    tau_firmware_bias: np.ndarray
    tau_comp: np.ndarray
    motion_lambda: float
    time_since_stop: np.ndarray
    firmware_state: np.ndarray
    is_moving: np.ndarray


def _predict_accepts_history(motion_model: Any) -> bool:
    try:
        params = inspect.signature(motion_model.predict).parameters
    except (TypeError, ValueError):
        return False
    return "history_features" in params or any(param.kind == inspect.Parameter.VAR_KEYWORD for param in params.values())


def _optional_positive_float(value: Any) -> float | None:
    if value is None:
        return None
    out = float(value)
    return out if out > 0.0 else None


def _clip_abs(values: np.ndarray, limit: float | None) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64).copy()
    if limit is None:
        return arr
    return np.clip(arr, -float(limit), float(limit))


class HybridTorqueCompensator:
    """Runtime hybrid compensator with stateful firmware-bias tracking."""

    kind = "hybrid"

    def __init__(
        self,
        *,
        static_model: StaticBiasModel,
        firmware_model: FirmwareStateModel,
        motion_model: PerJointMotionModel,
        metadata: dict[str, Any] | None = None,
    ):
        self.static_model = static_model
        self.firmware_model = firmware_model
        self.motion_model = motion_model
        self.metadata = metadata or {}
        limits = dict(self.metadata.get("runtime_limits", {}))
        self.motion_abs_max_nm = _optional_positive_float(limits.get("motion_abs_max_nm"))
        self.firmware_abs_max_nm = _optional_positive_float(limits.get("firmware_abs_max_nm"))
        self.comp_abs_max_nm = _optional_positive_float(limits.get("comp_abs_max_nm"))
        self.comp_rate_limit_nm_s = _optional_positive_float(limits.get("comp_rate_limit_nm_s"))
        self.tracker = KinematicFirmwareBiasTracker(firmware_model)
        self.history_steps = int(getattr(motion_model, "history_steps", 0))
        self._q_history: list[np.ndarray] = []
        self._qd_history: list[np.ndarray] = []
        self._qdd_history: list[np.ndarray] = []
        self._last_limited_comp: np.ndarray | None = None
        self._last_limit_timestamp: float | None = None
        self._motion_accepts_history = _predict_accepts_history(motion_model)

    @property
    def joint_count(self) -> int:
        return self.static_model.joint_count

    def reset(self) -> None:
        self.tracker.reset()
        self._q_history.clear()
        self._qd_history.clear()
        self._qdd_history.clear()
        self._last_limited_comp = None
        self._last_limit_timestamp = None

    def _history_features_for_current(self) -> np.ndarray:
        if self.history_steps <= 0:
            return np.zeros((1, 0), dtype=np.float32)
        width = self.joint_count * 3
        out = np.zeros((1, self.history_steps * width), dtype=np.float64)
        for lag in range(1, self.history_steps + 1):
            src = len(self._q_history) - lag
            if src < 0:
                continue
            start = (lag - 1) * width
            out[0, start : start + width] = np.concatenate(
                [self._q_history[src], self._qd_history[src], self._qdd_history[src]]
            )
        return out.astype(np.float32)

    def _remember_history(self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray) -> None:
        if self.history_steps <= 0:
            return
        self._q_history.append(np.asarray(q, dtype=np.float64).copy())
        self._qd_history.append(np.asarray(qd, dtype=np.float64).copy())
        self._qdd_history.append(np.asarray(qdd, dtype=np.float64).copy())
        max_len = self.history_steps
        del self._q_history[:-max_len]
        del self._qd_history[:-max_len]
        del self._qdd_history[:-max_len]

    def _predict_motion(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        qdd: np.ndarray,
        tau_model: np.ndarray,
        motion_lambda: float,
        time_since_stop: np.ndarray,
        history_features: np.ndarray,
    ) -> np.ndarray:
        if self._motion_accepts_history:
            return self.motion_model.predict(
                q,
                qd,
                qdd,
                tau_model,
                motion_lambda,
                time_since_stop,
                history_features=history_features,
            )
        return self.motion_model.predict(q, qd, qdd, tau_model, motion_lambda, time_since_stop)

    def _limit_compensation(
        self,
        tau_static: np.ndarray,
        tau_firmware: np.ndarray,
        tau_motion: np.ndarray,
        timestamp: float | None,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        limited_motion = _clip_abs(tau_motion, self.motion_abs_max_nm)
        limited_firmware = _clip_abs(tau_firmware, self.firmware_abs_max_nm)
        desired = _clip_abs(tau_static + limited_firmware + limited_motion, self.comp_abs_max_nm)
        if self.comp_rate_limit_nm_s is None or self._last_limited_comp is None:
            self._last_limited_comp = desired.copy()
            self._last_limit_timestamp = None if timestamp is None else float(timestamp)
            return limited_firmware, limited_motion, desired

        if timestamp is None or self._last_limit_timestamp is None:
            dt = 0.01
        else:
            dt = max(float(timestamp) - self._last_limit_timestamp, 0.0)
            if dt <= 0.0:
                dt = 0.01
        max_step = float(self.comp_rate_limit_nm_s) * dt
        limited = self._last_limited_comp + np.clip(desired - self._last_limited_comp, -max_step, max_step)
        self._last_limited_comp = limited.copy()
        self._last_limit_timestamp = None if timestamp is None else float(timestamp)
        return limited_firmware, limited_motion, limited

    def update(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        qdd: np.ndarray,
        tau_api: np.ndarray,
        tau_model: np.ndarray,
        timestamp: float | None = None,
    ) -> HybridTorqueEstimate:
        q_arr = np.asarray(q, dtype=np.float64)[: self.joint_count]
        qd_arr = np.asarray(qd, dtype=np.float64)[: self.joint_count]
        qdd_arr = np.asarray(qdd, dtype=np.float64)[: self.joint_count]
        tau_api_arr = np.asarray(tau_api, dtype=np.float64)[: self.joint_count]
        tau_model_arr = np.asarray(tau_model, dtype=np.float64)[: self.joint_count]

        tau_static = self.static_model.predict(q_arr)
        fw = self.tracker.update(q_arr, qd_arr, qdd_arr, timestamp=timestamp)
        history_features = self._history_features_for_current()
        tau_motion = self._predict_motion(
            q_arr,
            qd_arr,
            qdd_arr,
            tau_model_arr,
            fw.motion_lambda,
            fw.time_since_stop,
            history_features,
        )
        self._remember_history(q_arr, qd_arr, qdd_arr)
        tau_firmware, tau_motion, tau_comp = self._limit_compensation(tau_static, fw.bias, tau_motion, timestamp)
        tau_external = tau_api_arr - tau_model_arr - tau_comp
        return HybridTorqueEstimate(
            tau_external=tau_external,
            tau_static_bias=tau_static,
            tau_motion_comp=tau_motion,
            tau_firmware_bias=tau_firmware,
            tau_comp=tau_comp,
            motion_lambda=fw.motion_lambda,
            time_since_stop=fw.time_since_stop,
            firmware_state=fw.detected_levels,
            is_moving=fw.is_moving,
        )

    def predict_compensation(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        qdd: np.ndarray,
        tau_theory: np.ndarray | None = None,
    ) -> np.ndarray:
        """Compatibility API used by callers that only need compensation."""
        tau_model = (
            np.zeros(self.joint_count, dtype=np.float64)
            if tau_theory is None
            else np.asarray(tau_theory, dtype=np.float64)[: self.joint_count]
        )
        q_arr = np.asarray(q, dtype=np.float64)[: self.joint_count]
        qd_arr = np.asarray(qd, dtype=np.float64)[: self.joint_count]
        qdd_arr = np.asarray(qdd, dtype=np.float64)[: self.joint_count]
        tau_static = self.static_model.predict(q_arr)
        fw = self.tracker.update(q_arr, qd_arr, qdd_arr)
        history_features = self._history_features_for_current()
        tau_motion = self._predict_motion(
            q_arr,
            qd_arr,
            qdd_arr,
            tau_model,
            fw.motion_lambda,
            fw.time_since_stop,
            history_features,
        )
        self._remember_history(q_arr, qd_arr, qdd_arr)
        _, _, tau_comp = self._limit_compensation(tau_static, fw.bias, tau_motion, None)
        return tau_comp

    def checkpoint(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "static_model": self.static_model.to_dict(),
            "firmware_model": self.firmware_model.to_dict(),
            "motion_model": self.motion_model.to_checkpoint(),
            "metadata": self.metadata,
        }

    def save(self, path: str | Path) -> Path:
        out = Path(path).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        torch.save(self.checkpoint(), out)
        return out

    @classmethod
    def from_checkpoint(cls, checkpoint: dict[str, Any]) -> "HybridTorqueCompensator":
        return cls(
            static_model=StaticBiasModel.from_dict(checkpoint["static_model"]),
            firmware_model=FirmwareStateModel.from_dict(checkpoint["firmware_model"]),
            motion_model=PerJointMotionModel.from_checkpoint(checkpoint["motion_model"]),
            metadata=dict(checkpoint.get("metadata", {})),
        )

    @classmethod
    def load(cls, path: str | Path) -> "HybridTorqueCompensator":
        checkpoint = torch.load(Path(path).expanduser(), map_location="cpu", weights_only=False)
        if not isinstance(checkpoint, dict) or checkpoint.get("kind") != cls.kind:
            raise ValueError(f"not a hybrid compensation checkpoint: {path}")
        return cls.from_checkpoint(checkpoint)


def train_hybrid_compensator(
    *,
    q: np.ndarray,
    qd: np.ndarray,
    qdd: np.ndarray,
    tau_api: np.ndarray,
    tau_model: np.ndarray,
    timestamps: np.ndarray | None = None,
    static_q: np.ndarray | None = None,
    static_tau_api: np.ndarray | None = None,
    static_tau_model: np.ndarray | None = None,
    stop_q: np.ndarray | None = None,
    stop_qd: np.ndarray | None = None,
    stop_residual: np.ndarray | None = None,
    stop_tau_api: np.ndarray | None = None,
    stop_tau_model: np.ndarray | None = None,
    stop_timestamps: np.ndarray | None = None,
    static_alpha: float = 1.0,
    speed_threshold: float = np.deg2rad(2.0),
    epochs: int = 200,
    lr: float = 1e-3,
    hidden_dim: int = 64,
    seed: int = 7,
    history_steps: int = 3,
    firmware_min_level_gap: float = 1.0,
    firmware_default_decay_tau_s: float = 0.35,
    firmware_blend_alpha: float = 0.2,
    firmware_settle_lambda_threshold: float = 0.05,
    firmware_detect_ema_alpha: float = 0.05,
    firmware_j3_jump_size: float = 5.28,
    exclude_stop_window_s: float = 1.0,
    residual_clip_percentile: float = 98.0,
    motion_loss_kind: str = "huber",
    motion_huber_delta: float = 1.0,
    motion_target_clip_nm: float | None = 6.0,
    motion_abs_max_nm: float | None = 4.0,
    firmware_abs_max_nm: float | None = 2.0,
    comp_abs_max_nm: float | None = 8.0,
    comp_rate_limit_nm_s: float | None = 50.0,
    embodiment: dict[str, Any] | None = None,
) -> tuple[HybridTorqueCompensator, dict[str, Any]]:
    q_arr = np.asarray(q, dtype=np.float64)
    qd_arr = np.asarray(qd, dtype=np.float64)
    qdd_arr = np.asarray(qdd, dtype=np.float64)
    tau_api_arr = np.asarray(tau_api, dtype=np.float64)
    tau_model_arr = np.asarray(tau_model, dtype=np.float64)
    if q_arr.ndim != 2 or not (q_arr.shape == qd_arr.shape == qdd_arr.shape == tau_api_arr.shape == tau_model_arr.shape):
        raise ValueError("q, qd, qdd, tau_api, and tau_model must all have shape (N, J)")
    joint_count = int(q_arr.shape[1])

    residual = tau_api_arr - tau_model_arr
    global_speed = np.max(np.abs(qd_arr), axis=1)
    global_static_mask = global_speed <= float(speed_threshold)
    global_time_since_stop = derive_time_since_stop(global_speed[:, None], timestamps, float(speed_threshold))[:, 0]
    stop_exclude_mask = global_static_mask & (global_time_since_stop <= float(exclude_stop_window_s))
    residual_norm = np.linalg.norm(residual, axis=1)
    clip_percentile = float(residual_clip_percentile)
    if 0.0 < clip_percentile < 100.0:
        residual_norm_limit = float(np.nanpercentile(residual_norm, clip_percentile))
        residual_outlier_mask = residual_norm > residual_norm_limit
    else:
        residual_norm_limit = float("nan")
        residual_outlier_mask = np.zeros(q_arr.shape[0], dtype=bool)

    if static_q is None or static_tau_api is None or static_tau_model is None:
        static_mask = global_static_mask & ~stop_exclude_mask & ~residual_outlier_mask
        if np.count_nonzero(static_mask) < max(4, joint_count):
            static_model = StaticBiasModel.zeros(joint_count)
        else:
            static_model = StaticBiasModel.fit(q_arr[static_mask], residual[static_mask], alpha=static_alpha)
    else:
        static_residual = np.asarray(static_tau_api, dtype=np.float64) - np.asarray(static_tau_model, dtype=np.float64)
        static_model = StaticBiasModel.fit(np.asarray(static_q, dtype=np.float64), static_residual, alpha=static_alpha)

    residual_after_static = residual - static_model.predict(q_arr)
    if stop_residual is None and stop_q is not None and stop_tau_api is not None and stop_tau_model is not None:
        stop_residual = (
            np.asarray(stop_tau_api, dtype=np.float64)
            - np.asarray(stop_tau_model, dtype=np.float64)
            - static_model.predict(np.asarray(stop_q, dtype=np.float64))
        )

    if stop_qd is None or stop_residual is None:
        firmware_model = FirmwareStateModel.fit(
            residual_after_static,
            qd_arr,
            timestamps,
            speed_threshold=speed_threshold,
            min_level_gap=firmware_min_level_gap,
            default_decay_tau_s=firmware_default_decay_tau_s,
            blend_alpha=firmware_blend_alpha,
            settle_lambda_threshold=firmware_settle_lambda_threshold,
            detect_ema_alpha=firmware_detect_ema_alpha,
            j3_jump_size=firmware_j3_jump_size,
        )
    else:
        firmware_model = FirmwareStateModel.fit(
            np.asarray(stop_residual, dtype=np.float64),
            np.asarray(stop_qd, dtype=np.float64),
            stop_timestamps,
            speed_threshold=speed_threshold,
            min_level_gap=firmware_min_level_gap,
            default_decay_tau_s=firmware_default_decay_tau_s,
            blend_alpha=firmware_blend_alpha,
            settle_lambda_threshold=firmware_settle_lambda_threshold,
            detect_ema_alpha=firmware_detect_ema_alpha,
            j3_jump_size=firmware_j3_jump_size,
        )

    time_since_stop = derive_time_since_stop(qd_arr, timestamps, firmware_model.speed_threshold)
    motion_lambda = derive_motion_lambda(
        qd_arr,
        speed_threshold=firmware_model.speed_threshold,
        blend_alpha=firmware_model.blend_alpha,
    )
    firmware_bias = estimate_firmware_bias_sequence(firmware_model, q_arr, qd_arr, qdd_arr, timestamps)
    firmware_bias = _clip_abs(firmware_bias, _optional_positive_float(firmware_abs_max_nm))
    motion_target = residual_after_static - firmware_bias
    motion_target_norm = np.linalg.norm(motion_target, axis=1)
    if 0.0 < clip_percentile < 100.0:
        motion_target_norm_limit = float(np.nanpercentile(motion_target_norm, clip_percentile))
        motion_outlier_mask = motion_target_norm > motion_target_norm_limit
    else:
        motion_target_norm_limit = float("nan")
        motion_outlier_mask = np.zeros(q_arr.shape[0], dtype=bool)
    motion_train_mask = ~stop_exclude_mask & ~residual_outlier_mask & ~motion_outlier_mask
    motion_model, losses = train_per_joint_motion_model(
        q_arr,
        qd_arr,
        qdd_arr,
        tau_model_arr,
        motion_lambda,
        time_since_stop,
        motion_target,
        epochs=epochs,
        lr=lr,
        hidden_dim=hidden_dim,
        seed=seed,
        history_steps=history_steps,
        sample_weight=motion_train_mask.astype(np.float32),
        loss_kind=motion_loss_kind,
        huber_delta=motion_huber_delta,
        target_clip_abs=motion_target_clip_nm,
    )

    final_losses = [joint_losses[-1] if joint_losses else None for joint_losses in losses]
    metadata = {
        "joint_count": joint_count,
        "target": "tau_api_minus_tau_model",
        "components": ["static", "motion", "delayed_jump"],
        "jump_model": "kinematic_delayed_jump",
        "jump_eval_window_s": [0.0, 1.0],
        "speed_threshold": firmware_model.speed_threshold,
        "static_alpha": static_model.alpha,
        "static_rmse": static_model.residual_rmse.astype(float).tolist(),
        "motion_final_losses": final_losses,
        "motion_history_steps": motion_model.history_steps,
        "robust_training": {
            "exclude_stop_window_s": float(exclude_stop_window_s),
            "residual_clip_percentile": float(residual_clip_percentile),
            "residual_norm_limit": residual_norm_limit,
            "motion_target_norm_limit": motion_target_norm_limit,
            "static_fit_samples": int(np.count_nonzero(static_mask)) if 'static_mask' in locals() else None,
            "motion_fit_samples": int(np.count_nonzero(motion_train_mask)),
            "motion_loss_kind": str(motion_loss_kind),
            "motion_huber_delta": float(motion_huber_delta),
            "motion_target_clip_nm": None if motion_target_clip_nm is None else float(motion_target_clip_nm),
        },
        "runtime_limits": {
            "motion_abs_max_nm": None if motion_abs_max_nm is None else float(motion_abs_max_nm),
            "firmware_abs_max_nm": None if firmware_abs_max_nm is None else float(firmware_abs_max_nm),
            "comp_abs_max_nm": None if comp_abs_max_nm is None else float(comp_abs_max_nm),
            "comp_rate_limit_nm_s": None if comp_rate_limit_nm_s is None else float(comp_rate_limit_nm_s),
        },
        "firmware_fit": {
            "min_level_gap": float(firmware_min_level_gap),
            "default_decay_tau_s": float(firmware_default_decay_tau_s),
            "blend_alpha": firmware_model.blend_alpha,
            "settle_lambda_threshold": firmware_model.settle_lambda_threshold,
            "detect_ema_alpha": firmware_model.detect_ema_alpha,
            "j3_jump_size": firmware_model.j3_jump_size,
        },
    }
    if embodiment is not None:
        metadata["embodiment"] = embodiment
    return (
        HybridTorqueCompensator(
            static_model=static_model,
            firmware_model=firmware_model,
            motion_model=motion_model,
            metadata=metadata,
        ),
        metadata,
    )


def estimate_firmware_bias_sequence(
    firmware_model: FirmwareStateModel,
    q: np.ndarray,
    qd: np.ndarray,
    qdd: np.ndarray,
    timestamps: np.ndarray | None,
) -> np.ndarray:
    tracker = KinematicFirmwareBiasTracker(firmware_model)
    q_arr = np.asarray(q, dtype=np.float64)
    qd_arr = np.asarray(qd, dtype=np.float64)
    qdd_arr = np.asarray(qdd, dtype=np.float64)
    if timestamps is None:
        ts = np.arange(qd_arr.shape[0], dtype=np.float64) * 0.01
    else:
        ts = np.asarray(timestamps, dtype=np.float64)
    out = np.zeros_like(qd_arr)
    for idx in range(qd_arr.shape[0]):
        out[idx] = tracker.update(q_arr[idx], qd_arr[idx], qdd_arr[idx], timestamp=float(ts[idx])).bias
    return out
