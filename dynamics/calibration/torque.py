"""Torque replay and logging mode."""

from __future__ import annotations

from pathlib import Path
import threading
import time

import numpy as np

from dynamics.backends import create_backend
from dynamics.model import TheoreticalModel
from dynamics.resolver import resolve_robot

from .compensation import load_compensation
from .io import LowLatencyDifferentiator, extract_matrix, latest_parquet, make_record, read_parquet, timestamped_path, write_parquet


def resolve_traj_path(path: str | Path | None, traj_dir: str | Path = "dynamics/calibration/traj") -> Path:
    if path:
        out = Path(path).expanduser()
    else:
        found = latest_parquet(traj_dir)
        if found is None:
            raise FileNotFoundError(f"no trajectory parquet found in {traj_dir}; run --mode traj first")
        out = found
    if not out.exists():
        raise FileNotFoundError(f"trajectory file not found: {out}")
    return out


def collect_torque_data(
    config: dict,
    *,
    traj_path: str | Path | None = None,
    output_dir: str | Path = "dynamics/calibration/torque",
    model_path: str | Path | None = None,
) -> Path:
    robot_name = str(config.get("robot_name", "robot"))
    joint_count = int(config.get("joint_count", 6))
    traj_file = resolve_traj_path(traj_path)
    traj_df = read_parquet(traj_file)
    q_traj = extract_matrix(traj_df, "q", joint_count)
    timestamps = traj_df["timestamp"].to_numpy(dtype=np.float64) if "timestamp" in traj_df else None

    robot = resolve_robot(config["urdf_path"], name=robot_name, payload=config.get("payload"))
    comp = load_compensation(model_path) if model_path else None
    backend = create_backend(config)
    records: list[dict] = []
    diff = LowLatencyDifferentiator(joint_count)

    backend.connect()
    try:
        with TheoreticalModel(robot) as model:
            replay_thread = threading.Thread(
                target=backend.replay_joint_positions,
                kwargs={
                    "q_traj_rad": q_traj,
                    "timestamps": timestamps,
                    "speed_deg_s": float(config.get("replay_speed_deg_s", 20)),
                    "acc_deg_s2": float(config.get("replay_acc_deg_s2", 200)),
                },
                daemon=True,
            )
            replay_thread.start()
            start = time.monotonic()
            ts0 = float(timestamps[0]) if timestamps is not None and len(timestamps) else 0.0
            for idx in range(len(q_traj)):
                if timestamps is not None and idx > 0:
                    target = float(timestamps[idx]) - ts0
                    delay = target - (time.monotonic() - start)
                    if delay > 0:
                        time.sleep(delay)
                sample = backend.read_sample()
                qd, qdd = diff.update(sample.timestamp, sample.q, sample.qd)
                tau_api = sample.tau_api if sample.tau_api is not None else np.zeros(joint_count)
                tau_theory = model.estimate_joint_torque(sample.q, qd, qdd)
                if comp is None:
                    tau_comp = np.zeros(joint_count)
                else:
                    tau_comp = comp.predict_compensation(sample.q, qd, qdd, tau_theory=tau_theory)
                tau_error = tau_api - tau_theory - tau_comp
                records.append(
                    make_record(
                        timestamp=sample.timestamp,
                        robot=robot_name,
                        joint_count=joint_count,
                        mode="torque",
                        q=sample.q,
                        qd=qd,
                        qdd=qdd,
                        tau_api=tau_api,
                        tau_theory=tau_theory,
                        tau_comp=tau_comp,
                        tau_error=tau_error,
                        source_file=str(traj_file),
                    )
                )
            replay_thread.join(timeout=2.0)
    finally:
        backend.close()

    out = timestamped_path(output_dir, robot_name, "torque")
    path = write_parquet(records, out)
    print(f"[TORQUE] saved {len(records)} samples to {path}")
    return path
