"""Trajectory recording mode."""

from __future__ import annotations

from pathlib import Path
import time

import numpy as np

from dynamics.backends import create_backend

from .io import LowLatencyDifferentiator, make_record, timestamped_path, write_parquet
from .workspace import estimate_workspace_bounds, generate_random_workspace_trajectory


TRAJ_KINDS = ("drag", "workspace", "all")


def _capture_teach_records(
    config: dict,
    *,
    duration_s: float | None = None,
    hz: float | None = None,
    teach_sensitivity: int | None = None,
    traj_kind: str,
    prompt: str,
) -> tuple[str, int, list[dict]]:
    backend = create_backend(config)
    robot = str(config.get("robot_name", "robot"))
    joint_count = int(config.get("joint_count", 6))
    sample_hz = float(hz or config.get("sampling_hz", 100))
    dt = 1.0 / sample_hz
    diff = LowLatencyDifferentiator(joint_count)
    records: list[dict] = []

    backend.connect()
    try:
        backend.enter_teach_mode(sensitivity=teach_sensitivity)
        print(prompt)
        start = time.monotonic()
        next_t = time.monotonic()
        while duration_s is None or (time.monotonic() - start) < duration_s:
            sample = backend.read_sample()
            qd, qdd = diff.update(sample.timestamp, sample.q, sample.qd)
            records.append(
                make_record(
                    timestamp=sample.timestamp,
                    robot=robot,
                    joint_count=joint_count,
                    mode="traj",
                    q=sample.q,
                    qd=qd,
                    qdd=qdd,
                    traj_kind=traj_kind,
                )
            )
            next_t += dt
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()
    except KeyboardInterrupt:
        print("\n[TRAJ] stopping trajectory recording")
    finally:
        try:
            backend.exit_teach_mode()
        finally:
            backend.close()
    return robot, joint_count, records


def record_drag_trajectory(
    config: dict,
    *,
    output_dir: str | Path = "dynamics/calibration/traj",
    duration_s: float | None = None,
    hz: float | None = None,
    teach_sensitivity: int | None = None,
) -> Path:
    robot, _, records = _capture_teach_records(
        config,
        duration_s=duration_s,
        hz=hz,
        teach_sensitivity=teach_sensitivity,
        traj_kind="drag",
        prompt="[TRAJ:drag] teach mode enabled; move the arm by hand. Press Ctrl+C to stop.",
    )
    out = timestamped_path(output_dir, robot, "traj_drag")
    path = write_parquet(records, out)
    print(f"[TRAJ:drag] saved {len(records)} samples to {path}")
    return path


def record_workspace_trajectory(
    config: dict,
    *,
    output_dir: str | Path = "dynamics/calibration/traj",
    duration_s: float | None = None,
    hz: float | None = None,
    teach_sensitivity: int | None = None,
    point_count: int = 20,
    margin_ratio: float = 0.05,
    speed_deg_s: float = 30.0,
    seed: int = 7,
) -> Path:
    robot, joint_count, samples = _capture_teach_records(
        config,
        duration_s=duration_s,
        hz=hz,
        teach_sensitivity=teach_sensitivity,
        traj_kind="workspace_sample",
        prompt="[TRAJ:workspace] teach mode enabled; push the arm through safe boundary poses. Press Ctrl+C to generate.",
    )
    q_samples = np.asarray([[row[f"q_{idx}"] for idx in range(1, joint_count + 1)] for row in samples], dtype=np.float64)
    bounds = estimate_workspace_bounds(q_samples, margin_ratio=margin_ratio)
    sample_hz = float(hz or config.get("sampling_hz", 100))
    q_traj, rel_timestamps = generate_random_workspace_trajectory(
        bounds,
        point_count=point_count,
        hz=sample_hz,
        speed_deg_s=speed_deg_s,
        seed=seed,
        start_q=q_samples[-1],
    )

    diff = LowLatencyDifferentiator(joint_count)
    start_time = time.time()
    records: list[dict] = []
    for rel_ts, q in zip(rel_timestamps, q_traj):
        timestamp = start_time + float(rel_ts)
        qd, qdd = diff.update(timestamp, q)
        records.append(
            make_record(
                timestamp=timestamp,
                robot=robot,
                joint_count=joint_count,
                mode="traj",
                q=q,
                qd=qd,
                qdd=qdd,
                traj_kind="workspace",
            )
        )

    out = timestamped_path(output_dir, robot, "traj_workspace")
    path = write_parquet(records, out)
    print(f"[TRAJ:workspace] saved {len(records)} generated samples to {path}")
    return path


def record_trajectories(
    config: dict,
    *,
    output_dir: str | Path = "dynamics/calibration/traj",
    duration_s: float | None = None,
    hz: float | None = None,
    teach_sensitivity: int | None = None,
    traj_kind: str = "all",
    workspace_points: int = 20,
    workspace_margin_ratio: float = 0.05,
    workspace_speed_deg_s: float = 30.0,
    workspace_seed: int = 7,
) -> Path | list[Path]:
    if traj_kind == "drag":
        return record_drag_trajectory(
            config,
            output_dir=output_dir,
            duration_s=duration_s,
            hz=hz,
            teach_sensitivity=teach_sensitivity,
        )
    if traj_kind == "workspace":
        return record_workspace_trajectory(
            config,
            output_dir=output_dir,
            duration_s=duration_s,
            hz=hz,
            teach_sensitivity=teach_sensitivity,
            point_count=workspace_points,
            margin_ratio=workspace_margin_ratio,
            speed_deg_s=workspace_speed_deg_s,
            seed=workspace_seed,
        )
    if traj_kind == "all":
        return [
            record_drag_trajectory(
                config,
                output_dir=output_dir,
                duration_s=duration_s,
                hz=hz,
                teach_sensitivity=teach_sensitivity,
            ),
            record_workspace_trajectory(
                config,
                output_dir=output_dir,
                duration_s=duration_s,
                hz=hz,
                teach_sensitivity=teach_sensitivity,
                point_count=workspace_points,
                margin_ratio=workspace_margin_ratio,
                speed_deg_s=workspace_speed_deg_s,
                seed=workspace_seed,
            ),
        ]
    raise ValueError(f"traj_kind must be one of {', '.join(TRAJ_KINDS)}")


def record_trajectory(
    config: dict,
    *,
    output_dir: str | Path = "dynamics/calibration/traj",
    duration_s: float | None = None,
    hz: float | None = None,
    teach_sensitivity: int | None = None,
) -> Path:
    return record_drag_trajectory(
        config,
        output_dir=output_dir,
        duration_s=duration_s,
        hz=hz,
        teach_sensitivity=teach_sensitivity,
    )
