"""Torque replay and logging mode."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Sequence
import threading
import time

import numpy as np

from dynamics.backends import create_backend
from dynamics.model import TheoreticalModel
from dynamics.resolver import resolve_robot

from .io import LowLatencyDifferentiator, extract_matrix, latest_parquet, make_record, read_parquet, timestamped_path, write_parquet
from .runtime import estimate_torque_sample


def _latest_kind_parquet(directory: str | Path, traj_kind: str) -> Path | None:
    files = sorted(
        Path(directory).expanduser().glob(f"*_traj_{traj_kind}_*.parquet"),
        key=lambda p: (p.stat().st_mtime, p.name),
        reverse=True,
    )
    return files[0] if files else None


def resolve_traj_paths(
    paths: str | Path | Sequence[str | Path] | None,
    traj_dir: str | Path = "dynamics/calibration/traj",
) -> list[Path]:
    if paths:
        values = [paths] if isinstance(paths, (str, Path)) else list(paths)
        out = [Path(path).expanduser() for path in values]
    else:
        drag = _latest_kind_parquet(traj_dir, "drag")
        workspace = _latest_kind_parquet(traj_dir, "workspace")
        if drag is None and workspace is None:
            found = latest_parquet(traj_dir)
            if found is None:
                raise FileNotFoundError(f"no trajectory parquet found in {traj_dir}; run --mode traj first")
            out = [found]
        elif drag is None or workspace is None:
            missing = "drag" if drag is None else "workspace"
            raise FileNotFoundError(f"missing latest {missing} trajectory in {traj_dir}; run --mode traj --traj-kind all")
        else:
            out = [drag, workspace]

    missing_files = [path for path in out if not path.exists()]
    if missing_files:
        raise FileNotFoundError(f"trajectory file not found: {missing_files[0]}")
    return out


def resolve_traj_path(path: str | Path | None, traj_dir: str | Path = "dynamics/calibration/traj") -> Path:
    return resolve_traj_paths(path, traj_dir)[0]


def _sanitize_scalar_metadata(metadata: dict[str, Any] | None) -> dict[str, Any]:
    if not metadata:
        return {}
    out: dict[str, Any] = {}
    for key, value in metadata.items():
        if not isinstance(key, str) or not key:
            continue
        if isinstance(value, np.generic):
            value = value.item()
        if isinstance(value, bool):
            out[key] = bool(value)
        elif isinstance(value, int):
            out[key] = int(value)
        elif isinstance(value, float):
            if np.isfinite(value):
                out[key] = float(value)
        elif isinstance(value, str):
            out[key] = value
    return out


def collect_torque_data(
    config: dict,
    *,
    traj_path: str | Path | Sequence[str | Path] | None = None,
    traj_dir: str | Path = "dynamics/calibration/traj",
    output_dir: str | Path = "dynamics/calibration/torque",
    model_path: str | Path | None = None,
    replay_speed_deg_s: float | None = None,
    replay_acc_deg_s2: float | None = None,
    metadata: dict[str, Any] | None = None,
) -> Path:
    robot_name = str(config.get("robot_name", "robot"))
    joint_count = int(config.get("joint_count", 6))
    traj_files = resolve_traj_paths(traj_path, traj_dir=traj_dir)
    torque_cfg = config.get("torque", {})
    if not isinstance(torque_cfg, dict):
        torque_cfg = {}
    replay_speed = float(
        replay_speed_deg_s
        if replay_speed_deg_s is not None
        else torque_cfg.get("replay_speed_deg_s", config.get("replay_speed_deg_s", 30))
    )
    replay_acc = float(
        replay_acc_deg_s2
        if replay_acc_deg_s2 is not None
        else torque_cfg.get("replay_acc_deg_s2", config.get("replay_acc_deg_s2", 200))
    )
    base_metadata = _sanitize_scalar_metadata(metadata)
    base_metadata.setdefault("replay_speed_deg_s", replay_speed)
    base_metadata.setdefault("replay_acc_deg_s2", replay_acc)

    robot = resolve_robot(config["urdf_path"], name=robot_name, payload=config.get("payload"))
    if model_path:
        from .compensation import load_compensation_for_config

        comp = load_compensation_for_config(model_path, config)
    else:
        comp = None
    backend = create_backend(config)
    records: list[dict] = []

    backend.connect()
    try:
        with TheoreticalModel(robot) as model:
            for traj_file in traj_files:
                traj_df = read_parquet(traj_file)
                q_traj = extract_matrix(traj_df, "q", joint_count)
                timestamps = traj_df["timestamp"].to_numpy(dtype=np.float64) if "timestamp" in traj_df else None
                diff = LowLatencyDifferentiator(joint_count)
                if comp is not None and hasattr(comp, "reset"):
                    comp.reset()
                replay_errors: list[BaseException] = []

                def _run_replay() -> None:
                    try:
                        backend.replay_joint_positions(
                            q_traj_rad=q_traj,
                            timestamps=timestamps,
                            speed_deg_s=replay_speed,
                            acc_deg_s2=replay_acc,
                        )
                    except BaseException as exc:
                        replay_errors.append(exc)

                replay_thread = threading.Thread(target=_run_replay)
                replay_thread.start()
                start = time.monotonic()
                ts0 = float(timestamps[0]) if timestamps is not None and len(timestamps) else 0.0
                for idx in range(len(q_traj)):
                    if timestamps is not None and idx > 0:
                        target = float(timestamps[idx]) - ts0
                        delay = target - (time.monotonic() - start)
                        if delay > 0:
                            time.sleep(delay)
                    if replay_errors:
                        raise RuntimeError("replay thread failed") from replay_errors[0]
                    sample = backend.read_sample()
                    qd, qdd = diff.update(sample.timestamp, sample.q, sample.qd)
                    estimate = estimate_torque_sample(
                        sample=sample,
                        qd=qd,
                        qdd=qdd,
                        joint_count=joint_count,
                        model=model,
                        compensator=comp,
                    )
                    row = make_record(
                        timestamp=sample.timestamp,
                        robot=robot_name,
                        joint_count=joint_count,
                        mode="torque",
                        q=sample.q,
                        qd=qd,
                        qdd=qdd,
                        tau_api=estimate.tau_api,
                        tau_theory=estimate.tau_theory,
                        tau_comp=estimate.tau_comp,
                        tau_error=estimate.tau_error,
                        tau_static_bias=estimate.tau_static_bias,
                        tau_motion_comp=estimate.tau_motion_comp,
                        tau_firmware_bias=estimate.tau_firmware_bias,
                        tau_external=estimate.tau_external,
                        time_since_stop=estimate.time_since_stop,
                        firmware_state=estimate.firmware_state,
                        motion_lambda=estimate.motion_lambda,
                        is_moving=estimate.is_moving,
                        source_file=str(traj_file),
                    )
                    row.update(base_metadata)
                    records.append(row)
                replay_thread.join()
                if replay_errors:
                    raise RuntimeError("replay thread failed") from replay_errors[0]
    finally:
        backend.close()

    out = timestamped_path(output_dir, robot_name, "torque")
    path = write_parquet(records, out)
    print(f"[TORQUE] saved {len(records)} samples from {len(traj_files)} trajectories to {path}")
    return path
