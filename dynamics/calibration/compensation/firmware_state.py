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
    direction_levels: np.ndarray | None = None
    delay_s: np.ndarray | None = None
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
        direction_levels = _default_direction_levels(levels)
        return cls(
            levels=levels,
            decay_tau_s=np.full(joint_count, 0.35, dtype=np.float64),
            direction_levels=direction_levels,
            delay_s=np.zeros(joint_count, dtype=np.float64),
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
        delay = np.zeros(joint_count, dtype=np.float64)
        direction_levels = np.zeros((joint_count, 3), dtype=np.float64)
        time_since_stop = derive_time_since_stop(qd_arr, ts, speed_threshold)
        previous_direction = derive_previous_motion_direction(qd_arr, speed_threshold)

        for joint in range(joint_count):
            values = residual_arr[static_mask[:, joint], joint]
            if values.size < 4:
                default = DEFAULT_BIAS_LEVELS.get(joint + 1, (0.0, 0.0))
                levels.append(np.unique(np.asarray(default, dtype=np.float64)))
            else:
                levels.append(_fit_levels_1d(values, min_level_gap=min_level_gap))
            direction_levels[joint] = _fit_direction_levels_1d(
                residual_arr[:, joint],
                static_mask[:, joint],
                time_since_stop[:, joint],
                previous_direction[:, joint],
                levels[joint],
                joint_index=joint,
            )
            decay[joint] = _estimate_decay_tau(
                residual_arr[:, joint],
                time_since_stop[:, joint],
                levels[joint],
                default=float(default_decay_tau_s),
            )
            delay[joint] = _estimate_delay_s(
                residual_arr[:, joint],
                time_since_stop[:, joint],
                direction_levels[joint],
                previous_direction[:, joint],
            )

        return cls(
            levels=levels,
            decay_tau_s=decay,
            direction_levels=direction_levels,
            delay_s=delay,
            speed_threshold=float(speed_threshold),
        )

    def nearest_level(self, joint: int, value: float) -> float:
        arr = self.levels[int(joint)]
        if arr.size == 0:
            return 0.0
        idx = int(np.argmin(np.abs(arr - float(value))))
        return float(arr[idx])

    def level_for_direction(self, joint: int, direction: float) -> float:
        """Select a fitted jump level from previous motion direction.

        Direction bins are -1, 0, +1.  This is the runtime level selector used
        by the kinematic delayed-jump tracker; it does not inspect tau_api.
        """
        if self.direction_levels is not None:
            dirs = np.asarray(self.direction_levels, dtype=np.float64)
            if dirs.shape == (self.joint_count, 3):
                idx = int(np.sign(direction)) + 1
                return float(dirs[int(joint), idx])

        arr = self.levels[int(joint)]
        if arr.size == 0:
            return 0.0
        if arr.size == 1:
            return float(arr[0])
        return float(arr[-1] if direction >= 0 else arr[0])

    def kernel_value(self, joint: int, time_since_stop: float) -> float:
        delay = 0.0 if self.delay_s is None else float(np.asarray(self.delay_s, dtype=np.float64)[int(joint)])
        if float(time_since_stop) <= delay:
            return 0.0
        tau = max(float(self.decay_tau_s[int(joint)]), 1e-3)
        return float(1.0 - np.exp(-(float(time_since_stop) - delay) / tau))

    def to_dict(self) -> dict[str, Any]:
        return {
            "levels": [arr.astype(np.float32) for arr in self.levels],
            "decay_tau_s": self.decay_tau_s.astype(np.float32),
            "direction_levels": None if self.direction_levels is None else self.direction_levels.astype(np.float32),
            "delay_s": None if self.delay_s is None else self.delay_s.astype(np.float32),
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
            direction_levels=None
            if data.get("direction_levels") is None
            else np.asarray(data["direction_levels"], dtype=np.float64),
            delay_s=None if data.get("delay_s") is None else np.asarray(data["delay_s"], dtype=np.float64),
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


class KinematicFirmwareBiasTracker:
    """Delayed jump tracker driven only by q/qd/qdd history and timestamps."""

    def __init__(self, model: FirmwareStateModel):
        self.model = model
        self.joint_count = model.joint_count
        self.motion_lambda = 0.0
        self.prev_timestamp: float | None = None
        self.time_since_stop = np.zeros(self.joint_count, dtype=np.float64)
        self.time_since_motion_start = np.zeros(self.joint_count, dtype=np.float64)
        self.was_moving = np.zeros(self.joint_count, dtype=bool)
        self.previous_motion_direction = np.zeros(self.joint_count, dtype=np.float64)
        self.last_motion_direction = np.zeros(self.joint_count, dtype=np.float64)
        self.detected_levels = np.array([model.level_for_direction(j, 0.0) for j in range(self.joint_count)], dtype=np.float64)
        self.pre_stop_q = np.zeros(self.joint_count, dtype=np.float64)
        self.pre_stop_qd = np.zeros(self.joint_count, dtype=np.float64)
        self.pre_stop_qdd = np.zeros(self.joint_count, dtype=np.float64)

    def reset(self) -> None:
        self.motion_lambda = 0.0
        self.prev_timestamp = None
        self.time_since_stop[:] = 0.0
        self.time_since_motion_start[:] = 0.0
        self.was_moving[:] = False
        self.previous_motion_direction[:] = 0.0
        self.last_motion_direction[:] = 0.0
        self.detected_levels[:] = [self.model.level_for_direction(j, 0.0) for j in range(self.joint_count)]
        self.pre_stop_q[:] = 0.0
        self.pre_stop_qd[:] = 0.0
        self.pre_stop_qdd[:] = 0.0

    def update(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        qdd: np.ndarray,
        timestamp: float | None = None,
    ) -> FirmwareStateEstimate:
        q_arr = np.asarray(q, dtype=np.float64)[: self.joint_count]
        qd_arr = np.asarray(qd, dtype=np.float64)[: self.joint_count]
        qdd_arr = np.asarray(qdd, dtype=np.float64)[: self.joint_count]
        if q_arr.shape != (self.joint_count,) or qd_arr.shape != (self.joint_count,) or qdd_arr.shape != (self.joint_count,):
            raise ValueError(f"q, qd, and qdd must have shape ({self.joint_count},)")

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
                direction = float(np.sign(qd_arr[joint]))
                if not self.was_moving[joint]:
                    self.time_since_motion_start[joint] = 0.0
                else:
                    self.time_since_motion_start[joint] += dt
                self.was_moving[joint] = True
                self.time_since_stop[joint] = 0.0
                self.last_motion_direction[joint] = direction
                self.previous_motion_direction[joint] = direction
                self.pre_stop_q[joint] = q_arr[joint]
                self.pre_stop_qd[joint] = qd_arr[joint]
                self.pre_stop_qdd[joint] = qdd_arr[joint]
                self.detected_levels[joint] = self.model.level_for_direction(joint, direction)
                continue

            if self.was_moving[joint]:
                direction = self.last_motion_direction[joint]
                self.previous_motion_direction[joint] = direction
                self.time_since_stop[joint] = 0.0
                self.time_since_motion_start[joint] = 0.0
                self.was_moving[joint] = False
                self.detected_levels[joint] = self.model.level_for_direction(joint, direction)
            else:
                self.time_since_stop[joint] += dt

            bias[joint] = self.detected_levels[joint] * self.model.kernel_value(joint, self.time_since_stop[joint])

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
        prev_moving = np.abs(qd_arr[i - 1]) > float(speed_threshold)
        out[i] = np.where(moving | prev_moving, 0.0, out[i - 1] + dt)
    return out


def derive_previous_motion_direction(qd: np.ndarray, speed_threshold: float) -> np.ndarray:
    qd_arr = np.asarray(qd, dtype=np.float64)
    if qd_arr.ndim != 2:
        raise ValueError(f"qd must have shape (N, J), got {qd_arr.shape}")
    out = np.zeros_like(qd_arr, dtype=np.float64)
    last = np.zeros(qd_arr.shape[1], dtype=np.float64)
    for idx, row in enumerate(qd_arr):
        moving = np.abs(row) > float(speed_threshold)
        signs = np.sign(row)
        last = np.where(moving, signs, last)
        out[idx] = last
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


def _default_direction_levels(levels: list[np.ndarray]) -> np.ndarray:
    out = np.zeros((len(levels), 3), dtype=np.float64)
    for joint, arr in enumerate(levels):
        values = np.asarray(arr, dtype=np.float64)
        if values.size == 0:
            continue
        if values.size == 1:
            out[joint, :] = float(values[0])
        else:
            out[joint] = [float(values[0]), 0.0, float(values[-1])]
    return out


def _fit_direction_levels_1d(
    values: np.ndarray,
    static_mask: np.ndarray,
    time_since_stop: np.ndarray,
    previous_direction: np.ndarray,
    levels: np.ndarray,
    *,
    joint_index: int,
) -> np.ndarray:
    defaults = _default_direction_levels([levels])[0]
    out = defaults.copy()
    stop_window = static_mask & (time_since_stop >= 0.2) & (time_since_stop <= 1.0)
    for direction, col in [(-1.0, 0), (0.0, 1), (1.0, 2)]:
        mask = stop_window & (previous_direction == direction)
        if np.count_nonzero(mask) >= 3:
            out[col] = float(np.median(values[mask]))

    if np.allclose(out, 0.0):
        default_pair = DEFAULT_BIAS_LEVELS.get(joint_index + 1, (0.0, 0.0))
        out[:] = _default_direction_levels([np.unique(np.asarray(default_pair, dtype=np.float64))])[0]
    return out


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


def _estimate_delay_s(
    values: np.ndarray,
    time_since_stop: np.ndarray,
    direction_levels: np.ndarray,
    previous_direction: np.ndarray,
) -> float:
    vals = np.asarray(values, dtype=np.float64)
    tss = np.asarray(time_since_stop, dtype=np.float64)
    prev = np.roll(tss, 1)
    prev[0] = 0.0
    starts = np.flatnonzero((tss > 0.0) & (prev == 0.0))
    candidates: list[float] = []
    for start in starts:
        end = start + 1
        while end < vals.shape[0] and tss[end] > 0.0 and tss[end] <= 1.0:
            end += 1
        if end - start < 3:
            continue
        direction = int(np.sign(previous_direction[start])) + 1
        level = float(direction_levels[direction])
        if abs(level) < 0.2:
            continue
        segment = vals[start:end]
        times = tss[start:end]
        reached = np.flatnonzero(np.abs(segment) >= 0.1 * abs(level))
        if reached.size:
            candidates.append(float(times[int(reached[0])]))
    if not candidates:
        return 0.0
    return float(np.clip(np.median(candidates), 0.0, 0.5))
