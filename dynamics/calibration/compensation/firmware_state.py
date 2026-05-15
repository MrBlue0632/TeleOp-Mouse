"""Firmware/current-bias state model for xArm torque reports."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


DEFAULT_BIAS_LEVELS = {
    1: (0.0000, 0.0000),
    2: (-4.5056, 4.0151),
    3: (-2.4267, 3.3561),
    4: (0.0857, 0.0857),
    5: (-1.9928, 0.9617),
    6: (0.1845, 0.1845),
}


@dataclass
class FirmwareStateModel:
    """Per-joint firmware bias levels and decay constants."""

    levels: list[np.ndarray]
    decay_tau_s: np.ndarray
    speed_threshold: float = np.deg2rad(2.0)
    blend_alpha: float = 0.2
    settle_lambda_threshold: float = 0.05
    detect_ema_alpha: float = 0.05
    j3_jump_size: float = 5.28

    @property
    def joint_count(self) -> int:
        return len(self.levels)

    @classmethod
    def defaults(cls, joint_count: int = 6, *, speed_threshold: float = np.deg2rad(2.0)) -> "FirmwareStateModel":
        levels: list[np.ndarray] = []
        for joint in range(joint_count):
            pair = DEFAULT_BIAS_LEVELS.get(joint + 1, (0.0, 0.0))
            arr = np.unique(np.asarray(pair, dtype=np.float64))
            levels.append(arr)
        return cls(
            levels=levels,
            decay_tau_s=np.full(joint_count, 0.35, dtype=np.float64),
            speed_threshold=float(speed_threshold),
        )

    @classmethod
    def fit(
        cls,
        residual: np.ndarray,
        qd: np.ndarray,
        timestamps: np.ndarray | None,
        *,
        speed_threshold: float = np.deg2rad(2.0),
        min_level_gap: float = 1.0,
        default_decay_tau_s: float = 0.35,
    ) -> "FirmwareStateModel":
        residual_arr = np.asarray(residual, dtype=np.float64)
        qd_arr = np.asarray(qd, dtype=np.float64)
        if residual_arr.ndim != 2 or residual_arr.shape != qd_arr.shape:
            raise ValueError(f"residual and qd must both have shape (N, J), got {residual_arr.shape} and {qd_arr.shape}")
        n, joint_count = residual_arr.shape
        if timestamps is None:
            ts = np.arange(n, dtype=np.float64) * 0.01
        else:
            ts = np.asarray(timestamps, dtype=np.float64)
            if ts.shape != (n,):
                raise ValueError(f"timestamps must have shape ({n},), got {ts.shape}")

        static_mask = np.abs(qd_arr) <= float(speed_threshold)
        levels: list[np.ndarray] = []
        decay = np.full(joint_count, float(default_decay_tau_s), dtype=np.float64)
        time_since_stop = derive_time_since_stop(qd_arr, ts, speed_threshold)

        for joint in range(joint_count):
            values = residual_arr[static_mask[:, joint], joint]
            if values.size < 4:
                default = DEFAULT_BIAS_LEVELS.get(joint + 1, (0.0, 0.0))
                levels.append(np.unique(np.asarray(default, dtype=np.float64)))
            else:
                levels.append(_fit_levels_1d(values, min_level_gap=min_level_gap))
            decay[joint] = _estimate_decay_tau(
                residual_arr[:, joint],
                time_since_stop[:, joint],
                levels[joint],
                default=float(default_decay_tau_s),
            )

        return cls(levels=levels, decay_tau_s=decay, speed_threshold=float(speed_threshold))

    def nearest_level(self, joint: int, value: float) -> float:
        arr = self.levels[int(joint)]
        if arr.size == 0:
            return 0.0
        idx = int(np.argmin(np.abs(arr - float(value))))
        return float(arr[idx])

    def to_dict(self) -> dict[str, Any]:
        return {
            "levels": [arr.astype(np.float32) for arr in self.levels],
            "decay_tau_s": self.decay_tau_s.astype(np.float32),
            "speed_threshold": self.speed_threshold,
            "blend_alpha": self.blend_alpha,
            "settle_lambda_threshold": self.settle_lambda_threshold,
            "detect_ema_alpha": self.detect_ema_alpha,
            "j3_jump_size": self.j3_jump_size,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "FirmwareStateModel":
        return cls(
            levels=[np.asarray(arr, dtype=np.float64) for arr in data["levels"]],
            decay_tau_s=np.asarray(data["decay_tau_s"], dtype=np.float64),
            speed_threshold=float(data.get("speed_threshold", np.deg2rad(2.0))),
            blend_alpha=float(data.get("blend_alpha", 0.2)),
            settle_lambda_threshold=float(data.get("settle_lambda_threshold", 0.05)),
            detect_ema_alpha=float(data.get("detect_ema_alpha", 0.05)),
            j3_jump_size=float(data.get("j3_jump_size", 5.28)),
        )


@dataclass
class FirmwareStateEstimate:
    bias: np.ndarray
    motion_lambda: float
    time_since_stop: np.ndarray
    detected_levels: np.ndarray
    is_moving: np.ndarray

    @property
    def firmware_state(self) -> np.ndarray:
        return self.detected_levels


class FirmwareBiasTracker:
    """Runtime per-joint state tracker for stop-settle firmware bias."""

    def __init__(self, model: FirmwareStateModel):
        self.model = model
        self.joint_count = model.joint_count
        self.motion_lambda = 0.0
        self.prev_timestamp: float | None = None
        self.time_since_stop = np.zeros(self.joint_count, dtype=np.float64)
        self.was_moving = np.ones(self.joint_count, dtype=bool)
        self.stop_bias0 = np.zeros(self.joint_count, dtype=np.float64)
        self.detected_levels = np.array([model.nearest_level(j, 0.0) for j in range(self.joint_count)], dtype=np.float64)
        self.residual_ema = np.zeros(self.joint_count, dtype=np.float64)
        self.prev_residual = np.zeros(self.joint_count, dtype=np.float64)
        self.ema_initialized = np.zeros(self.joint_count, dtype=bool)

    def reset(self) -> None:
        self.prev_timestamp = None
        self.time_since_stop[:] = 0.0
        self.was_moving[:] = True
        self.stop_bias0[:] = 0.0
        self.detected_levels[:] = [self.model.nearest_level(j, 0.0) for j in range(self.joint_count)]
        self.residual_ema[:] = 0.0
        self.prev_residual[:] = 0.0
        self.ema_initialized[:] = False
        self.motion_lambda = 0.0

    def update(self, qd: np.ndarray, residual_after_static: np.ndarray, timestamp: float | None = None) -> FirmwareStateEstimate:
        qd_arr = np.asarray(qd, dtype=np.float64)[: self.joint_count]
        residual = np.asarray(residual_after_static, dtype=np.float64)[: self.joint_count]
        if qd_arr.shape != (self.joint_count,) or residual.shape != (self.joint_count,):
            raise ValueError(f"qd and residual must have shape ({self.joint_count},)")

        if timestamp is None:
            dt = 0.0 if self.prev_timestamp is None else 0.01
        elif self.prev_timestamp is None:
            dt = 0.0
            self.prev_timestamp = float(timestamp)
        else:
            dt = max(float(timestamp) - self.prev_timestamp, 0.0)
            self.prev_timestamp = float(timestamp)

        moving = np.abs(qd_arr) > self.model.speed_threshold
        global_moving = float(bool(np.any(moving)))
        a = self.model.blend_alpha
        self.motion_lambda = (1.0 - a) * self.motion_lambda + a * global_moving

        bias = np.zeros(self.joint_count, dtype=np.float64)
        for joint in range(self.joint_count):
            if moving[joint]:
                self.time_since_stop[joint] = 0.0
                self.was_moving[joint] = True
                self.ema_initialized[joint] = False
                bias[joint] = 0.0
                continue

            if self.was_moving[joint]:
                self.stop_bias0[joint] = residual[joint]
                self.detected_levels[joint] = self.model.nearest_level(joint, residual[joint])
                self.residual_ema[joint] = residual[joint]
                self.ema_initialized[joint] = True
                self.was_moving[joint] = False
                self.time_since_stop[joint] = 0.0
            else:
                self.time_since_stop[joint] += dt
                if joint == 2 and abs(residual[joint] - self.prev_residual[joint]) > 0.5 * self.model.j3_jump_size:
                    self.stop_bias0[joint] = residual[joint]
                    self.time_since_stop[joint] = 0.0
                    self.residual_ema[joint] = residual[joint]
                    self.detected_levels[joint] = self.model.nearest_level(joint, residual[joint])
                if self.motion_lambda < self.model.settle_lambda_threshold:
                    if not self.ema_initialized[joint]:
                        self.residual_ema[joint] = residual[joint]
                        self.ema_initialized[joint] = True
                    else:
                        alpha = self.model.detect_ema_alpha
                        self.residual_ema[joint] += alpha * (residual[joint] - self.residual_ema[joint])
                    self.detected_levels[joint] = self.model.nearest_level(joint, self.residual_ema[joint])

            tau = max(float(self.model.decay_tau_s[joint]), 1e-3)
            settled = self.detected_levels[joint]
            bias[joint] = settled + (self.stop_bias0[joint] - settled) * np.exp(-self.time_since_stop[joint] / tau)
            self.prev_residual[joint] = residual[joint]

        return FirmwareStateEstimate(
            bias=bias,
            motion_lambda=float(self.motion_lambda),
            time_since_stop=self.time_since_stop.copy(),
            detected_levels=self.detected_levels.copy(),
            is_moving=moving.copy(),
        )


def derive_time_since_stop(qd: np.ndarray, timestamps: np.ndarray | None, speed_threshold: float) -> np.ndarray:
    qd_arr = np.asarray(qd, dtype=np.float64)
    if qd_arr.ndim != 2:
        raise ValueError(f"qd must have shape (N, J), got {qd_arr.shape}")
    n, joint_count = qd_arr.shape
    if timestamps is None:
        ts = np.arange(n, dtype=np.float64) * 0.01
    else:
        ts = np.asarray(timestamps, dtype=np.float64)
    out = np.zeros((n, joint_count), dtype=np.float64)
    for i in range(1, n):
        dt = max(float(ts[i] - ts[i - 1]), 0.0)
        moving = np.abs(qd_arr[i]) > float(speed_threshold)
        out[i] = np.where(moving, 0.0, out[i - 1] + dt)
    return out


def derive_motion_lambda(qd: np.ndarray, *, speed_threshold: float, blend_alpha: float = 0.2) -> np.ndarray:
    qd_arr = np.asarray(qd, dtype=np.float64)
    out = np.zeros(qd_arr.shape[0], dtype=np.float64)
    lam = 0.0
    for i, row in enumerate(qd_arr):
        is_moving = float(np.max(np.abs(row)) > float(speed_threshold))
        lam = (1.0 - float(blend_alpha)) * lam + float(blend_alpha) * is_moving
        out[i] = lam
    return out


def _fit_levels_1d(values: np.ndarray, *, min_level_gap: float) -> np.ndarray:
    vals = np.asarray(values, dtype=np.float64)
    if vals.size < 6:
        return np.asarray([float(np.mean(vals))], dtype=np.float64)
    c0, c1 = np.percentile(vals, [25, 75])
    for _ in range(20):
        dist0 = np.abs(vals - c0)
        dist1 = np.abs(vals - c1)
        mask = dist1 < dist0
        if np.any(~mask):
            c0 = float(np.mean(vals[~mask]))
        if np.any(mask):
            c1 = float(np.mean(vals[mask]))
    centers = np.sort(np.asarray([c0, c1], dtype=np.float64))
    if abs(float(centers[1] - centers[0])) < float(min_level_gap):
        return np.asarray([float(np.mean(vals))], dtype=np.float64)
    return centers


def _estimate_decay_tau(values: np.ndarray, time_since_stop: np.ndarray, levels: np.ndarray, *, default: float) -> float:
    vals = np.asarray(values, dtype=np.float64)
    tss = np.asarray(time_since_stop, dtype=np.float64)
    prev = np.roll(tss, 1)
    prev[0] = 0.0
    starts = np.flatnonzero((tss > 0.0) & (prev == 0.0))
    candidates: list[float] = []
    for start in starts:
        if start + 3 >= vals.shape[0]:
            continue
        end = start + 1
        while end < vals.shape[0] and tss[end] > 0.0:
            end += 1
        segment = vals[start:end]
        times = tss[start:end]
        if segment.size < 4:
            continue
        settled = levels[int(np.argmin(np.abs(levels - segment[-1])))] if levels.size else 0.0
        initial_gap = segment[0] - settled
        if abs(initial_gap) < 0.2:
            continue
        target = settled + initial_gap / np.e
        idx = int(np.argmin(np.abs(segment - target)))
        if times[idx] > 0:
            candidates.append(float(times[idx]))
    if not candidates:
        return float(default)
    return float(np.clip(np.median(candidates), 0.05, 3.0))
