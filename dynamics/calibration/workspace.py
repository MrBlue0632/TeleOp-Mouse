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


def _minimum_jerk(u: np.ndarray) -> np.ndarray:
    return 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5


def interpolate_waypoints_minimum_jerk(
    waypoints: np.ndarray,
    *,
    hz: float,
    speed_deg_s: float,
    accel_deg_s2: float,
    hold_s: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Interpolate radian waypoints with a minimum-jerk profile and speed/acc caps."""
    q_waypoints = np.asarray(waypoints, dtype=np.float64)
    if q_waypoints.ndim != 2 or q_waypoints.shape[0] == 0:
        raise ValueError(f"waypoints must have shape (N, J) with N > 0, got {q_waypoints.shape}")
    sample_hz = float(hz)
    speed_rad_s = float(np.deg2rad(speed_deg_s))
    accel_rad_s2 = float(np.deg2rad(accel_deg_s2))
    hold = max(0.0, float(hold_s))
    if sample_hz <= 0.0:
        raise ValueError("hz must be > 0")
    if speed_rad_s <= 0.0:
        raise ValueError("speed_deg_s must be > 0")
    if accel_rad_s2 <= 0.0:
        raise ValueError("accel_deg_s2 must be > 0")

    dt = 1.0 / sample_hz
    rows = [q_waypoints[0].copy()]
    timestamps = [0.0]
    current = q_waypoints[0].copy()
    for target in q_waypoints[1:]:
        delta = target - current
        max_delta = float(np.max(np.abs(delta)))
        if max_delta <= 1e-12:
            duration = dt
        else:
            # q(s)=10s^3-15s^4+6s^5 has max velocity 1.875/T and
            # max acceleration about 5.8/T^2, so these are conservative.
            duration_speed = 1.875 * max_delta / speed_rad_s
            duration_accel = float(np.sqrt(5.8 * max_delta / accel_rad_s2))
            duration = max(duration_speed, duration_accel, dt)
        steps = max(1, int(np.ceil(duration / dt)))
        for step in range(1, steps + 1):
            u = np.array(step / steps, dtype=np.float64)
            rows.append(current + delta * float(_minimum_jerk(u)))
            timestamps.append(timestamps[-1] + dt)
        current = target.copy()
        hold_steps = int(round(hold * sample_hz))
        for _ in range(hold_steps):
            rows.append(current.copy())
            timestamps.append(timestamps[-1] + dt)
    return np.vstack(rows), np.asarray(timestamps, dtype=np.float64)


def generate_safe_joint_trajectory(
    home_q_rad: np.ndarray,
    *,
    duration_s: float = 180.0,
    hz: float = 100.0,
    amplitude_deg: np.ndarray | None = None,
    speed_deg_s: float = 15.0,
    accel_deg_s2: float = 60.0,
    hold_s: float = 3.0,
    waypoint_count: int = 18,
    seed: int = 7,
    joint_limits_deg: tuple[np.ndarray, np.ndarray] | None = None,
    limit_buffer_deg: float = 10.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Generate a conservative no-contact joint trajectory around home.

    The output is in radians and is intended to be replayed later by torque mode;
    generating it does not move the robot.
    """
    home = np.asarray(home_q_rad, dtype=np.float64).reshape(-1)
    joint_count = int(home.shape[0])
    if joint_count <= 0:
        raise ValueError("home_q_rad must contain at least one joint")
    sample_hz = float(hz)
    if sample_hz <= 0.0:
        raise ValueError("hz must be > 0")
    duration = max(float(duration_s), 2.0 * float(hold_s) + 1.0 / sample_hz)
    amp_deg = np.asarray(
        amplitude_deg if amplitude_deg is not None else [20.0, 15.0, 15.0, 25.0, 20.0, 20.0],
        dtype=np.float64,
    )
    if amp_deg.size < joint_count:
        amp_deg = np.pad(amp_deg, (0, joint_count - amp_deg.size), mode="edge")
    amp_rad = np.deg2rad(np.abs(amp_deg[:joint_count]))

    rng = np.random.default_rng(int(seed))
    waypoints = [home.copy()]
    if joint_limits_deg is not None:
        low_deg, high_deg = joint_limits_deg
        low = np.deg2rad(np.asarray(low_deg, dtype=np.float64)[:joint_count] + float(limit_buffer_deg))
        high = np.deg2rad(np.asarray(high_deg, dtype=np.float64)[:joint_count] - float(limit_buffer_deg))
    else:
        low = home - amp_rad
        high = home + amp_rad
    low = np.maximum(low, home - amp_rad)
    high = np.minimum(high, home + amp_rad)
    for idx in range(max(1, int(waypoint_count))):
        # Structured random waypoints keep all joints excited without leaving the envelope.
        phase = 2.0 * np.pi * (idx + 1) / max(2, int(waypoint_count))
        sinusoid = np.sin(phase + np.arange(joint_count) * 0.73)
        jitter = rng.uniform(-0.35, 0.35, size=joint_count)
        waypoint = home + amp_rad * np.clip(0.75 * sinusoid + jitter, -1.0, 1.0)
        waypoints.append(np.clip(waypoint, low, high))
    waypoints.append(home.copy())

    q, ts = interpolate_waypoints_minimum_jerk(
        np.vstack(waypoints),
        hz=sample_hz,
        speed_deg_s=speed_deg_s,
        accel_deg_s2=accel_deg_s2,
        hold_s=hold_s,
    )
    # Treat duration_s as a minimum duration. Never truncate the generated path,
    # because the final waypoint returns to home for a predictable safe stop.
    if ts[-1] < duration:
        hold_steps = int(round((duration - ts[-1]) * sample_hz))
        if hold_steps > 0:
            q = np.vstack([q, np.repeat(q[-1][None, :], hold_steps, axis=0)])
            ts = np.concatenate([ts, ts[-1] + np.arange(1, hold_steps + 1, dtype=np.float64) / sample_hz])
    if not np.all(np.isfinite(q)):
        raise ValueError("generated trajectory contains non-finite values")
    return q, ts
