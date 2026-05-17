"""Workspace-bounded trajectory generation helpers."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class WorkspaceBounds:
    lower: np.ndarray
    upper: np.ndarray

    @property
    def joint_count(self) -> int:
        return int(self.lower.shape[0])


def estimate_workspace_bounds(q_samples: np.ndarray, *, margin_ratio: float = 0.05) -> WorkspaceBounds:
    """Estimate per-joint safe bounds from hand-guided samples."""
    q_arr = np.asarray(q_samples, dtype=np.float64)
    if q_arr.ndim != 2 or q_arr.shape[0] == 0:
        raise ValueError(f"q_samples must have shape (N, J) with N > 0, got {q_arr.shape}")
    ratio = float(margin_ratio)
    if ratio < 0.0 or ratio >= 0.5:
        raise ValueError("margin_ratio must be in [0.0, 0.5)")

    raw_lower = np.min(q_arr, axis=0)
    raw_upper = np.max(q_arr, axis=0)
    margin = (raw_upper - raw_lower) * ratio
    return WorkspaceBounds(lower=raw_lower + margin, upper=raw_upper - margin)


def generate_random_workspace_trajectory(
    bounds: WorkspaceBounds,
    *,
    point_count: int = 20,
    hz: float = 100.0,
    speed_deg_s: float = 30.0,
    seed: int = 7,
    start_q: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate random joint-space waypoints and interpolate under a speed cap."""
    if int(point_count) < 1:
        raise ValueError("point_count must be >= 1")
    rng = np.random.default_rng(int(seed))
    waypoints = rng.uniform(bounds.lower, bounds.upper, size=(int(point_count), bounds.joint_count))
    if start_q is not None:
        start = np.clip(np.asarray(start_q, dtype=np.float64)[: bounds.joint_count], bounds.lower, bounds.upper)
        waypoints = np.vstack([start, waypoints])
    return interpolate_waypoints(waypoints, hz=hz, speed_deg_s=speed_deg_s)


def interpolate_waypoints(
    waypoints: np.ndarray,
    *,
    hz: float,
    speed_deg_s: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Linearly interpolate waypoints with synchronized per-joint velocity limits."""
    q_waypoints = np.asarray(waypoints, dtype=np.float64)
    if q_waypoints.ndim != 2 or q_waypoints.shape[0] == 0:
        raise ValueError(f"waypoints must have shape (N, J) with N > 0, got {q_waypoints.shape}")
    sample_hz = float(hz)
    speed_rad_s = float(np.deg2rad(speed_deg_s))
    if sample_hz <= 0.0:
        raise ValueError("hz must be > 0")
    if speed_rad_s <= 0.0:
        raise ValueError("speed_deg_s must be > 0")

    dt = 1.0 / sample_hz
    rows = [q_waypoints[0].copy()]
    timestamps = [0.0]
    current = q_waypoints[0].copy()
    for target in q_waypoints[1:]:
        delta = target - current
        duration = float(np.max(np.abs(delta)) / speed_rad_s)
        steps = max(1, int(np.ceil(duration / dt)))
        for step in range(1, steps + 1):
            rows.append(current + delta * (step / steps))
            timestamps.append(timestamps[-1] + dt)
        current = target.copy()
    return np.vstack(rows), np.asarray(timestamps, dtype=np.float64)
