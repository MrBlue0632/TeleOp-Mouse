"""Trajectory recording mode."""

from __future__ import annotations

from pathlib import Path
import time

from dynamics.backends import create_backend

from .io import LowLatencyDifferentiator, make_record, timestamped_path, write_parquet


def record_trajectory(
    config: dict,
    *,
    output_dir: str | Path = "dynamics/calibration/traj",
    duration_s: float | None = None,
    hz: float | None = None,
    teach_sensitivity: int | None = None,
) -> Path:
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
        print("[TRAJ] teach mode enabled; move the arm by hand. Press Ctrl+C to stop.")
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

    out = timestamped_path(output_dir, robot, "traj")
    path = write_parquet(records, out)
    print(f"[TRAJ] saved {len(records)} samples to {path}")
    return path
