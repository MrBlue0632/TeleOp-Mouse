"""Validation metrics for torque compensation breakdowns."""

from __future__ import annotations

from typing import Any

import numpy as np

from .compensation.firmware_state import derive_time_since_stop


def _as_matrix(values: np.ndarray, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.ndim != 2:
        raise ValueError(f"{name} must have shape (N, J), got {arr.shape}")
    return arr


def _zeros_like(reference: np.ndarray, values: np.ndarray | None, name: str) -> np.ndarray:
    if values is None:
        return np.zeros_like(reference)
    arr = _as_matrix(values, name)
    if arr.shape != reference.shape:
        raise ValueError(f"{name} must have shape {reference.shape}, got {arr.shape}")
    return arr


def _rmse(error: np.ndarray, mask: np.ndarray | None = None) -> float:
    values = error if mask is None else error[mask]
    if values.size == 0:
        return float("nan")
    return float(np.sqrt(np.mean(values**2)))


def _per_joint_rmse(error: np.ndarray, mask: np.ndarray | None = None) -> list[float]:
    values = error if mask is None else error[mask]
    if values.size == 0:
        return [float("nan")] * int(error.shape[1])
    return np.sqrt(np.mean(values**2, axis=0)).astype(float).tolist()


def _metrics_for_prediction(
    tau_api: np.ndarray,
    prediction: np.ndarray,
    *,
    static_mask: np.ndarray,
    motion_mask: np.ndarray,
    stop_window_mask: np.ndarray,
) -> dict[str, Any]:
    error = tau_api - prediction
    return {
        "rmse": _rmse(error),
        "per_joint_rmse": _per_joint_rmse(error),
        "static_rmse": _rmse(error, static_mask),
        "motion_rmse": _rmse(error, motion_mask),
        "stop_0_1s_rmse": _rmse(error, stop_window_mask),
        "stop_0_1s_per_joint_rmse": _per_joint_rmse(error, stop_window_mask),
    }


def evaluate_torque_breakdown(
    *,
    qd: np.ndarray,
    tau_api: np.ndarray,
    tau_model: np.ndarray,
    tau_static: np.ndarray | None = None,
    tau_motion: np.ndarray | None = None,
    tau_jump_delay: np.ndarray | None = None,
    timestamps: np.ndarray | None = None,
    speed_threshold: float = np.deg2rad(2.0),
) -> dict[str, dict[str, Any]]:
    """Compare model-only and staged compensation predictions.

    The full prediction is:

    ``tau_model + tau_static + tau_motion + tau_jump_delay``.
    """
    qd_arr = _as_matrix(qd, "qd")
    tau_api_arr = _as_matrix(tau_api, "tau_api")
    tau_model_arr = _as_matrix(tau_model, "tau_model")
    if qd_arr.shape != tau_api_arr.shape or tau_model_arr.shape != tau_api_arr.shape:
        raise ValueError("qd, tau_api, and tau_model must have the same shape")

    static_arr = _zeros_like(tau_api_arr, tau_static, "tau_static")
    motion_arr = _zeros_like(tau_api_arr, tau_motion, "tau_motion")
    jump_arr = _zeros_like(tau_api_arr, tau_jump_delay, "tau_jump_delay")

    is_static = np.max(np.abs(qd_arr), axis=1) <= float(speed_threshold)
    is_motion = ~is_static
    time_since_stop = derive_time_since_stop(qd_arr, timestamps, speed_threshold)
    stop_window = is_static & (np.max(time_since_stop, axis=1) > 0.0) & (np.max(time_since_stop, axis=1) <= 1.0)

    predictions = {
        "model_only": tau_model_arr,
        "static": tau_model_arr + static_arr,
        "static_motion": tau_model_arr + static_arr + motion_arr,
        "static_motion_jump": tau_model_arr + static_arr + motion_arr + jump_arr,
    }
    metrics = {
        name: _metrics_for_prediction(
            tau_api_arr,
            prediction,
            static_mask=is_static,
            motion_mask=is_motion,
            stop_window_mask=stop_window,
        )
        for name, prediction in predictions.items()
    }
    metrics["full"] = metrics["static_motion_jump"]
    return metrics
