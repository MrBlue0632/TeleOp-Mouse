"""Speed-tiered trajectory replay, torque logging, and compensation retraining."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np
import pandas as pd

from dynamics.calibration.io import LowLatencyDifferentiator, extract_matrix, make_record, read_parquet, write_parquet
from dynamics.calibration.torque import collect_torque_data
from dynamics.calibration.train import train_from_torque_data
from dynamics.calibration.workspace import generate_safe_joint_trajectory
from dynamics.config import load_config

SPEED_TIERS = (15.0, 25.0, 35.0, 45.0)
DEFAULT_HF_REPOS = (
    "DorayakiLin/xarm6_pick_bread_lerobot",
    "DorayakiLin/xarm6_pick_oreo_lerobot",
    "DorayakiLin/xarm6_pick_sprite_lerobot",
    "DorayakiLin/xarm6_pick_coke_lerobot",
    "DorayakiLin/xarm6_pick_fanta_lerobot",
)
DEFAULT_AMPLITUDE_DEG = (8.0, 6.0, 6.0, 10.0, 8.0, 8.0)
DEFAULT_HF_MAX_STEP_DEG = 15.0
DEFAULT_OUTPUT_ROOT = Path("dynamics/calibration/speed_ladder_runs")
DEFAULT_OLD_MODEL = Path("dynamics/calibration/compensation/history_q_qd.pt")
DEFAULT_COMPENSATION_DIR = Path("dynamics/calibration/compensation")
MANIFEST_NAME = "manifest.jsonl"
STAGES = (
    "generate-local",
    "collect-local",
    "download-hf",
    "prepare-hf",
    "collect-hf",
    "merge",
    "train",
    "validate",
    "all",
)
REAL_ROBOT_STAGES = {"collect-local", "collect-hf", "all"}


@dataclass(frozen=True)
class TrajectoryEntry:
    path: Path
    trajectory_id: str
    speed_deg_s: float
    speed_tier: str
    source_kind: str
    source_repo: str = ""
    episode_index: int = -1
    seed: int = -1


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _repo_path(path: str | Path) -> Path:
    value = Path(path).expanduser()
    return value if value.is_absolute() else _repo_root() / value


def _speed_tier(speed_deg_s: float) -> str:
    return f"{int(round(float(speed_deg_s)))}deg_s"


def _speed_dir(speed_deg_s: float) -> str:
    return f"speed_{int(round(float(speed_deg_s)))}"


def slug_repo(repo_id: str) -> str:
    return repo_id.replace("/", "__")


def experiment_id_now() -> str:
    return "speed_ladder_" + time.strftime("%Y%m%d_%H%M%S")


def parse_float_csv(text: str) -> list[float]:
    values = [float(part.strip()) for part in str(text).split(",") if part.strip()]
    if not values or not all(math.isfinite(value) and value > 0.0 for value in values):
        raise argparse.ArgumentTypeError("value must contain positive comma-separated floats")
    return values


def parse_amplitude(text: str) -> list[float]:
    values = parse_float_csv(text)
    if len(values) != 6:
        raise argparse.ArgumentTypeError("--local-amplitude-deg must contain six values")
    return values


def ensure_root(root: str | Path) -> Path:
    out = _repo_path(root)
    out.mkdir(parents=True, exist_ok=True)
    return out


def append_manifest(root: str | Path, event: dict[str, Any]) -> None:
    out = ensure_root(root) / MANIFEST_NAME
    row = dict(event)
    row.setdefault("experiment_id", Path(root).name)
    row.setdefault("wall_time", time.strftime("%Y-%m-%dT%H:%M:%S%z"))
    with out.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(row, sort_keys=True) + "\n")


def load_manifest(root: str | Path) -> list[dict[str, Any]]:
    path = _repo_path(root) / MANIFEST_NAME
    if not path.exists():
        return []
    events: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                events.append(json.loads(line))
    return events


def _trajectory_metadata(
    *,
    root: str | Path,
    trajectory_id: str,
    speed_deg_s: float,
    source_kind: str,
    source_repo: str = "",
    episode_index: int = -1,
    seed: int = -1,
) -> dict[str, Any]:
    return {
        "experiment_id": Path(root).name,
        "source_kind": str(source_kind),
        "source_repo": str(source_repo),
        "episode_index": int(episode_index),
        "trajectory_id": str(trajectory_id),
        "speed_deg_s": float(speed_deg_s),
        "speed_tier": _speed_tier(speed_deg_s),
        "seed": int(seed),
    }


def _records_from_trajectory(
    *,
    q_traj_rad: np.ndarray,
    timestamps: np.ndarray,
    robot: str,
    joint_count: int,
    traj_kind: str,
    metadata: dict[str, Any],
) -> list[dict[str, Any]]:
    diff = LowLatencyDifferentiator(joint_count)
    records: list[dict[str, Any]] = []
    for timestamp, q in zip(np.asarray(timestamps, dtype=np.float64), np.asarray(q_traj_rad, dtype=np.float64)):
        qd, qdd = diff.update(float(timestamp), q)
        row = make_record(
            timestamp=float(timestamp),
            robot=robot,
            joint_count=joint_count,
            mode="traj",
            q=q,
            qd=qd,
            qdd=qdd,
            traj_kind=traj_kind,
        )
        row.update(metadata)
        records.append(row)
    return records


def local_trajectory_path(root: str | Path, speed_deg_s: float, trajectory_id: str) -> Path:
    return ensure_root(root) / "traj" / "local" / _speed_dir(speed_deg_s) / f"{trajectory_id}.parquet"


def hf_trajectory_path(root: str | Path, speed_deg_s: float, repo_id: str, trajectory_id: str) -> Path:
    return ensure_root(root) / "traj" / "hf" / _speed_dir(speed_deg_s) / slug_repo(repo_id) / f"{trajectory_id}.parquet"


def generate_local_trajectories(
    config: dict,
    root: str | Path,
    *,
    speeds: Sequence[float] = SPEED_TIERS,
    count: int = 100,
    duration_s: float = 20.0,
    hz: float = 100.0,
    amplitude_deg: Sequence[float] = DEFAULT_AMPLITUDE_DEG,
    accel_deg_s2: float = 60.0,
    hold_s: float = 0.0,
    waypoint_count: int = 8,
    seed_base: int = 0,
    resume: bool = True,
) -> list[Path]:
    root_path = ensure_root(root)
    robot = str(config.get("robot_name", "xarm6"))
    joint_count = int(config.get("joint_count", 6))
    if joint_count != 6:
        raise ValueError(f"speed ladder expects 6 joints, got {joint_count}")
    home_deg = np.asarray(config.get("home_joints_deg", np.zeros(joint_count)), dtype=np.float64)[:joint_count]
    if home_deg.shape != (joint_count,) or not np.all(np.isfinite(home_deg)):
        raise ValueError("config home_joints_deg must contain finite joint values")
    amp = np.asarray(amplitude_deg, dtype=np.float64)
    if amp.shape != (joint_count,) or not np.all(np.isfinite(amp)) or np.any(amp <= 0.0):
        raise ValueError("amplitude_deg must contain six positive finite values")

    generated: list[Path] = []
    for speed in speeds:
        for idx in range(int(count)):
            seed = int(seed_base) + int(round(float(speed) * 1000.0)) + idx
            trajectory_id = f"local_s{int(round(float(speed))):02d}_{idx:03d}"
            out = local_trajectory_path(root_path, speed, trajectory_id)
            if resume and out.exists():
                generated.append(out)
                continue
            q_traj, timestamps = generate_safe_joint_trajectory(
                np.deg2rad(home_deg),
                duration_s=float(duration_s),
                hz=float(hz),
                amplitude_deg=amp,
                speed_deg_s=float(speed),
                accel_deg_s2=float(accel_deg_s2),
                hold_s=max(0.0, float(hold_s)),
                waypoint_count=int(waypoint_count),
                seed=seed,
            )
            q_deg = np.rad2deg(q_traj)
            if np.nanmax(np.abs(q_deg - home_deg) - amp) > 1e-6:
                raise ValueError(f"generated trajectory {trajectory_id} exceeds local amplitude envelope")
            if not np.allclose(q_deg[-1], home_deg, atol=1e-6):
                raise ValueError(f"generated trajectory {trajectory_id} does not return home")
            metadata = _trajectory_metadata(
                root=root_path,
                trajectory_id=trajectory_id,
                speed_deg_s=float(speed),
                source_kind="local",
                seed=seed,
            )
            records = _records_from_trajectory(
                q_traj_rad=q_traj,
                timestamps=timestamps,
                robot=robot,
                joint_count=joint_count,
                traj_kind="safe_no_contact_speed_ladder",
                metadata=metadata,
            )
            path = write_parquet(records, out)
            append_manifest(
                root_path,
                {
                    "event": "trajectory_generated",
                    "path": str(path),
                    "rows": len(records),
                    **metadata,
                },
            )
            generated.append(path)
    return generated


def load_joint_limits_from_urdf(urdf_path: str | Path, joint_count: int, *, limit_buffer_deg: float = 5.0) -> tuple[np.ndarray, np.ndarray]:
    path = _repo_path(urdf_path)
    if not path.exists():
        raise FileNotFoundError(f"URDF not found: {path}")
    root = ET.parse(path).getroot()
    lower: list[float] = []
    upper: list[float] = []
    for joint in root.findall("joint"):
        if joint.attrib.get("type") == "fixed":
            continue
        limit = joint.find("limit")
        if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
            continue
        lower.append(float(limit.attrib["lower"]))
        upper.append(float(limit.attrib["upper"]))
        if len(lower) == int(joint_count):
            break
    if len(lower) < int(joint_count):
        raise ValueError(f"URDF {path} does not expose {joint_count} revolute joint limits")
    buffer_rad = math.radians(float(limit_buffer_deg))
    low = np.asarray(lower[:joint_count], dtype=np.float64) + buffer_rad
    high = np.asarray(upper[:joint_count], dtype=np.float64) - buffer_rad
    if np.any(low >= high):
        raise ValueError("joint limit buffer leaves an empty joint interval")
    return low, high


def _stack_vector_values(values: Iterable[Any], joint_count: int) -> np.ndarray:
    rows: list[np.ndarray] = []
    for value in values:
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
        if arr.size < int(joint_count):
            arr = np.pad(arr, (0, int(joint_count) - arr.size), constant_values=np.nan)
        rows.append(arr[: int(joint_count)])
    if not rows:
        return np.zeros((0, int(joint_count)), dtype=np.float64)
    return np.vstack(rows).astype(np.float64)


def extract_lerobot_joint_degrees(df: pd.DataFrame, joint_count: int = 6) -> tuple[np.ndarray, str]:
    if "action" in df.columns:
        return _stack_vector_values(df["action"].to_numpy(), joint_count), "action"
    if "observation.state" in df.columns:
        return _stack_vector_values(df["observation.state"].to_numpy(), joint_count), "observation.state"
    raise KeyError("HF LeRobot parquet must contain action or observation.state")


def validate_joint_trajectory(
    q_rad: np.ndarray,
    *,
    joint_limits: tuple[np.ndarray, np.ndarray],
    home_q_rad: np.ndarray | None = None,
    min_frames: int = 20,
    max_step_deg: float = DEFAULT_HF_MAX_STEP_DEG,
    start_home_tolerance_deg: float | None = 60.0,
) -> tuple[bool, str]:
    q = np.asarray(q_rad, dtype=np.float64)
    if q.ndim != 2 or q.shape[0] < int(min_frames):
        return False, "too_few_frames"
    if not np.all(np.isfinite(q)):
        return False, "non_finite"
    if q.shape[0] > 1:
        max_step = float(np.nanmax(np.abs(np.diff(np.rad2deg(q), axis=0))))
        if max_step > float(max_step_deg):
            return False, "joint_step_too_large"
    low, high = joint_limits
    if np.any(q < low[None, :] - 1e-12) or np.any(q > high[None, :] + 1e-12):
        return False, "joint_limit_buffer_violation"
    if home_q_rad is not None and start_home_tolerance_deg is not None:
        start_delta = np.abs(np.rad2deg(q[0] - np.asarray(home_q_rad, dtype=np.float64)))
        if float(np.nanmax(start_delta)) > float(start_home_tolerance_deg):
            return False, "start_too_far_from_home"
    return True, "ok"


def retime_joint_trajectory(q_rad: np.ndarray, *, speed_deg_s: float, hz: float = 100.0) -> tuple[np.ndarray, np.ndarray]:
    q = np.asarray(q_rad, dtype=np.float64)
    if q.ndim != 2 or q.shape[0] == 0:
        raise ValueError(f"q_rad must have shape (N, J) with N > 0, got {q.shape}")
    sample_hz = float(hz)
    speed_rad_s = math.radians(float(speed_deg_s))
    if sample_hz <= 0.0 or speed_rad_s <= 0.0:
        raise ValueError("hz and speed_deg_s must be > 0")
    dt = 1.0 / sample_hz
    rows = [q[0].copy()]
    timestamps = [0.0]
    current = q[0].copy()
    for target in q[1:]:
        delta = target - current
        duration = max(float(np.max(np.abs(delta))) / speed_rad_s, dt)
        steps = max(1, int(math.ceil(duration / dt)))
        for step in range(1, steps + 1):
            rows.append(current + delta * (step / steps))
            timestamps.append(timestamps[-1] + dt)
        current = target.copy()
    return np.vstack(rows), np.asarray(timestamps, dtype=np.float64)


def _hf_snapshot_dir(root: str | Path, repo_id: str) -> Path:
    return ensure_root(root) / "hf_snapshots" / slug_repo(repo_id)


def download_hf_datasets(
    root: str | Path,
    *,
    repos: Sequence[str] = DEFAULT_HF_REPOS,
    resume: bool = True,
) -> list[Path]:
    try:
        from huggingface_hub import snapshot_download
    except Exception as exc:  # pragma: no cover - depends on runtime env
        raise RuntimeError("huggingface_hub is required for download-hf") from exc

    paths: list[Path] = []
    for repo_id in repos:
        target = _hf_snapshot_dir(root, repo_id)
        if resume and target.exists() and any(target.rglob("*.parquet")):
            paths.append(target)
            continue
        target.mkdir(parents=True, exist_ok=True)
        snapshot_download(
            repo_id=repo_id,
            repo_type="dataset",
            local_dir=str(target),
            allow_patterns=["*.parquet", "data/**/*.parquet", "meta/*.json", "meta/*.jsonl", "meta/*.parquet"],
        )
        append_manifest(root, {"event": "hf_downloaded", "source_repo": repo_id, "path": str(target)})
        paths.append(target)
    return paths


def _hf_parquet_files(snapshot_dir: Path) -> list[Path]:
    files = sorted(snapshot_dir.glob("data/chunk-*/file-*.parquet"))
    if files:
        return files
    return sorted(path for path in snapshot_dir.rglob("*.parquet") if "/meta/" not in str(path))


def _read_hf_dataframe(snapshot_dir: Path) -> pd.DataFrame:
    files = _hf_parquet_files(snapshot_dir)
    if not files:
        raise FileNotFoundError(f"no HF parquet files found under {snapshot_dir}")
    return pd.concat([pd.read_parquet(path) for path in files], ignore_index=True)


def _episode_groups(df: pd.DataFrame) -> Iterable[tuple[int, pd.DataFrame]]:
    sort_cols = [col for col in ("episode_index", "frame_index", "index", "timestamp") if col in df.columns]
    ordered = df.sort_values(sort_cols) if sort_cols else df
    if "episode_index" not in ordered.columns:
        yield 0, ordered.reset_index(drop=True)
        return
    for episode_index, group in ordered.groupby("episode_index", sort=True):
        yield int(episode_index), group.reset_index(drop=True)


def prepare_hf_trajectories(
    config: dict,
    root: str | Path,
    *,
    repos: Sequence[str] = DEFAULT_HF_REPOS,
    speeds: Sequence[float] = SPEED_TIERS,
    hz: float = 100.0,
    min_frames: int = 20,
    max_step_deg: float = DEFAULT_HF_MAX_STEP_DEG,
    limit_buffer_deg: float = 5.0,
    start_home_tolerance_deg: float = 60.0,
    max_episodes: int | None = None,
    resume: bool = True,
) -> list[Path]:
    root_path = ensure_root(root)
    joint_count = int(config.get("joint_count", 6))
    robot = str(config.get("robot_name", "xarm6"))
    home_q = np.deg2rad(np.asarray(config.get("home_joints_deg", np.zeros(joint_count)), dtype=np.float64)[:joint_count])
    joint_limits = load_joint_limits_from_urdf(config["urdf_path"], joint_count, limit_buffer_deg=limit_buffer_deg)
    prepared: list[Path] = []

    for repo_id in repos:
        snapshot = _hf_snapshot_dir(root_path, repo_id)
        if not snapshot.exists():
            append_manifest(root_path, {"event": "hf_prepare_skipped", "source_repo": repo_id, "reason": "snapshot_missing"})
            continue
        df = _read_hf_dataframe(snapshot)
        valid_seen = 0
        for episode_index, group in _episode_groups(df):
            if max_episodes is not None and valid_seen >= int(max_episodes):
                break
            try:
                q_deg, source_column = extract_lerobot_joint_degrees(group, joint_count=joint_count)
            except Exception as exc:
                append_manifest(
                    root_path,
                    {"event": "hf_episode_skipped", "source_repo": repo_id, "episode_index": episode_index, "reason": type(exc).__name__},
                )
                continue
            q_raw = np.deg2rad(q_deg)
            ok, reason = validate_joint_trajectory(
                q_raw,
                joint_limits=joint_limits,
                home_q_rad=home_q,
                min_frames=min_frames,
                max_step_deg=max_step_deg,
                start_home_tolerance_deg=start_home_tolerance_deg,
            )
            if not ok:
                append_manifest(
                    root_path,
                    {"event": "hf_episode_skipped", "source_repo": repo_id, "episode_index": episode_index, "reason": reason},
                )
                continue
            valid_seen += 1
            for speed in speeds:
                trajectory_id = f"hf_{slug_repo(repo_id)}_ep{episode_index:05d}_s{int(round(float(speed))):02d}"
                out = hf_trajectory_path(root_path, speed, repo_id, trajectory_id)
                if resume and out.exists():
                    prepared.append(out)
                    continue
                q_traj, timestamps = retime_joint_trajectory(q_raw, speed_deg_s=float(speed), hz=float(hz))
                metadata = _trajectory_metadata(
                    root=root_path,
                    trajectory_id=trajectory_id,
                    speed_deg_s=float(speed),
                    source_kind="hf",
                    source_repo=repo_id,
                    episode_index=episode_index,
                    seed=-1,
                )
                metadata["hf_column"] = source_column
                records = _records_from_trajectory(
                    q_traj_rad=q_traj,
                    timestamps=timestamps,
                    robot=robot,
                    joint_count=joint_count,
                    traj_kind="hf_lerobot_replay_speed_ladder",
                    metadata=metadata,
                )
                path = write_parquet(records, out)
                append_manifest(
                    root_path,
                    {
                        "event": "trajectory_prepared",
                        "path": str(path),
                        "rows": len(records),
                        **metadata,
                    },
                )
                prepared.append(path)
    return prepared


def load_trajectory_index(root: str | Path) -> list[TrajectoryEntry]:
    events = load_manifest(root)
    by_path: dict[str, TrajectoryEntry] = {}
    for event in events:
        if event.get("event") not in {"trajectory_generated", "trajectory_prepared"}:
            continue
        path_text = event.get("path")
        if not path_text:
            continue
        path = _repo_path(path_text)
        if not path.exists():
            continue
        by_path[str(path)] = TrajectoryEntry(
            path=path,
            trajectory_id=str(event.get("trajectory_id", path.stem)),
            speed_deg_s=float(event.get("speed_deg_s", 0.0)),
            speed_tier=str(event.get("speed_tier", _speed_tier(float(event.get("speed_deg_s", 0.0))))),
            source_kind=str(event.get("source_kind", "")),
            source_repo=str(event.get("source_repo", "")),
            episode_index=int(event.get("episode_index", -1)),
            seed=int(event.get("seed", -1)),
        )
    return sorted(by_path.values(), key=lambda item: (item.source_kind, item.speed_deg_s, item.trajectory_id))


def _collected_trajectory_ids(root: str | Path) -> set[str]:
    return {str(event.get("trajectory_id")) for event in load_manifest(root) if event.get("event") == "torque_collected"}


def _torque_metadata(root: str | Path, entry: TrajectoryEntry) -> dict[str, Any]:
    return _trajectory_metadata(
        root=root,
        trajectory_id=entry.trajectory_id,
        speed_deg_s=entry.speed_deg_s,
        source_kind=entry.source_kind,
        source_repo=entry.source_repo,
        episode_index=entry.episode_index,
        seed=entry.seed,
    )


def _summarize_torque_file(torque_path: str | Path, traj_path: str | Path) -> dict[str, Any]:
    df = read_parquet(torque_path)
    traj_df = read_parquet(traj_path)
    joint_count = len([col for col in df.columns if col.startswith("q_")])
    summary: dict[str, Any] = {"samples": int(len(df))}
    if joint_count:
        tau_cols = [f"tau_api_{i}" for i in range(1, joint_count + 1)]
        if all(col in df.columns for col in tau_cols):
            tau_api = extract_matrix(df, "tau_api", joint_count)
            abs_tau = np.abs(tau_api)
            summary["tau_api_abs_max_nm"] = round(float(np.nanmax(abs_tau)), 4)
            summary["tau_api_abs_p95_nm"] = round(float(np.nanpercentile(abs_tau, 95)), 4)
        q_obs = extract_matrix(df, "q", joint_count)
        q_ref = extract_matrix(traj_df, "q", joint_count)
        n = min(len(q_obs), len(q_ref))
        if n > 0:
            err_deg = np.abs(np.rad2deg(q_obs[:n] - q_ref[:n]))
            summary["tracking_abs_max_deg"] = round(float(np.nanmax(err_deg)), 4)
            summary["tracking_abs_p95_deg"] = round(float(np.nanpercentile(err_deg, 95)), 4)
    return summary


def _require_real_robot_ack(stage: str, acknowledged: bool) -> None:
    if stage in REAL_ROBOT_STAGES and not acknowledged:
        raise SystemExit(
            "This stage moves the real robot. Re-run with --i-understand-real-robot-motion after clearing the workspace."
        )


def _confirm_speed(source_kind: str, speed: float, summary: dict[str, Any], dashboard_url: str) -> bool:
    print("============================================")
    print(f"Sanity replay complete: source={source_kind} speed={speed:.1f} deg/s")
    print(json.dumps(summary, indent=2, sort_keys=True))
    print(f"Dashboard/log reference: {dashboard_url}")
    print("Type exactly CONFIRM %d to collect the remaining trajectories for this speed." % int(round(speed)))
    return input("> ").strip() == f"CONFIRM {int(round(speed))}"


def collect_speed_ladder_torque(
    config: dict,
    root: str | Path,
    *,
    source_kind: str,
    speeds: Sequence[float] = SPEED_TIERS,
    replay_acc_deg_s2: float = 200.0,
    model_path: str | Path | None = None,
    dashboard_url: str = "http://127.0.0.1:8765/",
    acknowledged: bool = False,
    resume: bool = True,
) -> list[Path]:
    _require_real_robot_ack("collect-hf" if source_kind == "hf" else "collect-local", acknowledged)
    root_path = ensure_root(root)
    entries = [entry for entry in load_trajectory_index(root_path) if entry.source_kind == source_kind]
    collected = _collected_trajectory_ids(root_path) if resume else set()
    outputs: list[Path] = []
    for speed in speeds:
        speed_entries = [entry for entry in entries if math.isclose(entry.speed_deg_s, float(speed), rel_tol=0.0, abs_tol=1e-9)]
        speed_entries = [entry for entry in speed_entries if entry.trajectory_id not in collected]
        if not speed_entries:
            continue
        sanity = speed_entries[0]
        sanity_out = collect_one_torque_file(
            config,
            root_path,
            sanity,
            replay_acc_deg_s2=replay_acc_deg_s2,
            model_path=model_path,
        )
        outputs.append(sanity_out)
        summary = _summarize_torque_file(sanity_out, sanity.path)
        append_manifest(root_path, {"event": "sanity_complete", "torque_path": str(sanity_out), **_torque_metadata(root_path, sanity), **summary})
        if not _confirm_speed(source_kind, float(speed), summary, dashboard_url):
            append_manifest(root_path, {"event": "speed_not_confirmed", "source_kind": source_kind, "speed_deg_s": float(speed)})
            raise SystemExit(f"speed {speed:.1f} deg/s was not confirmed; stopping before remaining trajectories")
        append_manifest(root_path, {"event": "speed_confirmed", "source_kind": source_kind, "speed_deg_s": float(speed)})
        for entry in speed_entries[1:]:
            out = collect_one_torque_file(
                config,
                root_path,
                entry,
                replay_acc_deg_s2=replay_acc_deg_s2,
                model_path=model_path,
            )
            outputs.append(out)
    return outputs


def collect_one_torque_file(
    config: dict,
    root: str | Path,
    entry: TrajectoryEntry,
    *,
    replay_acc_deg_s2: float = 200.0,
    model_path: str | Path | None = None,
) -> Path:
    output_dir = ensure_root(root) / "torque" / entry.source_kind / _speed_dir(entry.speed_deg_s) / entry.trajectory_id
    try:
        path = collect_torque_data(
            config,
            traj_path=[entry.path],
            output_dir=output_dir,
            model_path=model_path,
            replay_speed_deg_s=entry.speed_deg_s,
            replay_acc_deg_s2=float(replay_acc_deg_s2),
            metadata=_torque_metadata(root, entry),
        )
    except Exception as exc:
        append_manifest(root, {"event": "torque_failed", "reason": type(exc).__name__, **_torque_metadata(root, entry)})
        raise
    append_manifest(root, {"event": "torque_collected", "torque_path": str(path), **_torque_metadata(root, entry)})
    return path


def _torque_files(root: str | Path) -> list[Path]:
    torque_root = ensure_root(root) / "torque"
    if not torque_root.exists():
        return []
    return sorted(path for path in torque_root.rglob("*.parquet") if "merged" not in path.parts)


def merge_torque_data(
    root: str | Path,
    *,
    valid_ratio: float = 0.2,
    seed: int = 7,
) -> dict[str, Path]:
    files = _torque_files(root)
    if not files:
        raise FileNotFoundError(f"no torque parquet files found under {ensure_root(root) / 'torque'}")
    df = pd.concat([read_parquet(path) for path in files], ignore_index=True)
    if "trajectory_id" not in df.columns:
        raise KeyError("merged torque data must contain trajectory_id")
    trajectory_ids = sorted(str(value) for value in df["trajectory_id"].dropna().unique())
    if not trajectory_ids:
        raise ValueError("no trajectory ids found in torque data")
    rng = np.random.default_rng(int(seed))
    shuffled = np.asarray(trajectory_ids, dtype=object)
    rng.shuffle(shuffled)
    valid_count = 0 if len(shuffled) == 1 else max(1, int(round(len(shuffled) * float(valid_ratio))))
    valid_ids = set(str(value) for value in shuffled[:valid_count])
    df["split"] = ["valid" if str(value) in valid_ids else "train" for value in df["trajectory_id"]]
    merged_dir = ensure_root(root) / "merged"
    merged_dir.mkdir(parents=True, exist_ok=True)
    merged_path = merged_dir / "speed_ladder_torque.parquet"
    train_path = merged_dir / "speed_ladder_train.parquet"
    valid_path = merged_dir / "speed_ladder_valid.parquet"
    df.to_parquet(merged_path, index=False)
    df[df["split"] == "train"].to_parquet(train_path, index=False)
    df[df["split"] == "valid"].to_parquet(valid_path, index=False)
    split_manifest = merged_dir / "split_manifest.json"
    split_manifest.write_text(
        json.dumps(
            {
                "trajectory_count": len(trajectory_ids),
                "train_count": int(len(set(trajectory_ids) - valid_ids)),
                "valid_count": int(len(valid_ids)),
                "valid_ids": sorted(valid_ids),
                "seed": int(seed),
            },
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    append_manifest(root, {"event": "torque_merged", "path": str(merged_path), "rows": int(len(df)), "files": len(files)})
    return {"merged": merged_path, "train": train_path, "valid": valid_path, "split_manifest": split_manifest}


def default_model_output_path(root: str | Path) -> Path:
    return _repo_path(DEFAULT_COMPENSATION_DIR) / f"history_q_qd_speed_ladder_{Path(root).name}.pt"


def train_speed_ladder_model(
    config: dict,
    root: str | Path,
    *,
    output_path: str | Path | None = None,
    epochs: int = 200,
    lr: float = 1e-3,
    hidden_dim: int = 64,
    seed: int = 7,
) -> Path:
    train_path = ensure_root(root) / "merged" / "speed_ladder_train.parquet"
    if not train_path.exists():
        raise FileNotFoundError(f"missing train split: {train_path}; run merge first")
    out = _repo_path(output_path) if output_path else default_model_output_path(root)
    if out.exists():
        raise FileExistsError(f"model output already exists: {out}")
    path = train_from_torque_data(
        config,
        data_path=train_path,
        output_path=out,
        model_kind="kinematic_history",
        epochs=int(epochs),
        lr=float(lr),
        hidden_dim=int(hidden_dim),
        seed=int(seed),
    )
    append_manifest(root, {"event": "model_trained", "path": str(path), "data_path": str(train_path)})
    return path


def _metric_block(error: np.ndarray) -> dict[str, Any]:
    arr = np.asarray(error, dtype=np.float64)
    if arr.size == 0:
        return {"rows": 0, "rmse_mean": float("nan"), "norm_p95": float("nan"), "norm_max": float("nan"), "per_joint_rmse": []}
    norm = np.linalg.norm(arr, axis=1)
    per_joint = np.sqrt(np.nanmean(arr * arr, axis=0))
    return {
        "rows": int(arr.shape[0]),
        "rmse_mean": round(float(np.nanmean(per_joint)), 6),
        "norm_p95": round(float(np.nanpercentile(norm, 95)), 6),
        "norm_max": round(float(np.nanmax(norm)), 6),
        "per_joint_rmse": np.round(per_joint, 6).astype(float).tolist(),
    }


def _predict_compensation(config: dict, df: pd.DataFrame, model_path: str | Path) -> np.ndarray:
    from dynamics.calibration.compensation import load_compensation_for_config

    joint_count = int(config.get("joint_count", 6))
    comp = load_compensation_for_config(model_path, config)
    q = extract_matrix(df, "q", joint_count)
    qd = extract_matrix(df, "qd", joint_count)
    qdd = extract_matrix(df, "qdd", joint_count)
    tau_model = extract_matrix(df, "tau_model", joint_count)
    out = np.zeros((len(df), joint_count), dtype=np.float64)
    group_col = "trajectory_id" if "trajectory_id" in df.columns else "source_file"
    for _, indices in df.groupby(group_col, sort=False).groups.items():
        if hasattr(comp, "reset"):
            comp.reset()
        for idx in list(indices):
            out[int(idx)] = np.asarray(comp.predict_compensation(q[int(idx)], qd[int(idx)], qdd[int(idx)], tau_theory=tau_model[int(idx)]), dtype=np.float64)[:joint_count]
    return out


def build_validation_report(
    config: dict,
    df: pd.DataFrame,
    *,
    model_paths: dict[str, str | Path | None],
) -> dict[str, Any]:
    joint_count = int(config.get("joint_count", 6))
    tau_api = extract_matrix(df, "tau_api", joint_count)
    tau_model = extract_matrix(df, "tau_model", joint_count)
    predictions: dict[str, np.ndarray] = {"model_only": np.zeros_like(tau_api)}
    for name, path in model_paths.items():
        if path and _repo_path(path).exists():
            predictions[name] = _predict_compensation(config, df, path)
    group_specs: list[tuple[str, pd.Series | None]] = [("all", None)]
    for col in ("speed_tier", "source_kind", "source_repo"):
        if col in df.columns:
            for value in sorted(str(item) for item in df[col].dropna().unique()):
                group_specs.append((f"{col}={value}", df[col].astype(str) == value))
    results: dict[str, dict[str, Any]] = {}
    for model_name, tau_comp in predictions.items():
        error = tau_api - tau_model - tau_comp
        model_results: dict[str, Any] = {}
        for group_name, mask in group_specs:
            group_error = error if mask is None else error[np.asarray(mask, dtype=bool)]
            model_results[group_name] = _metric_block(group_error)
        results[model_name] = model_results
    return {"rows": int(len(df)), "models": results}


def validate_speed_ladder_models(
    config: dict,
    root: str | Path,
    *,
    old_model_path: str | Path | None = DEFAULT_OLD_MODEL,
    new_model_path: str | Path | None = None,
) -> dict[str, Path]:
    valid_path = ensure_root(root) / "merged" / "speed_ladder_valid.parquet"
    if not valid_path.exists():
        raise FileNotFoundError(f"missing valid split: {valid_path}; run merge first")
    if new_model_path is None:
        new_model_path = default_model_output_path(root)
    df = read_parquet(valid_path)
    report = build_validation_report(
        config,
        df,
        model_paths={"old_model": old_model_path, "new_model": new_model_path},
    )
    out_dir = ensure_root(root) / "reports"
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / "validation_report.json"
    csv_path = out_dir / "validation_report.csv"
    json_path.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["model", "group", "rows", "rmse_mean", "norm_p95", "norm_max", "per_joint_rmse"])
        writer.writeheader()
        for model_name, groups in report["models"].items():
            for group_name, metrics in groups.items():
                writer.writerow({"model": model_name, "group": group_name, **metrics, "per_joint_rmse": json.dumps(metrics["per_joint_rmse"])})
    append_manifest(root, {"event": "models_validated", "json_path": str(json_path), "csv_path": str(csv_path)})
    return {"json": json_path, "csv": csv_path}


def resolve_experiment_root(args: argparse.Namespace) -> Path:
    if args.experiment_root:
        return ensure_root(args.experiment_root)
    output_root = _repo_path(args.output_root)
    if args.experiment_id:
        return ensure_root(output_root / args.experiment_id)
    if args.stage in {"generate-local", "download-hf", "all"}:
        return ensure_root(output_root / experiment_id_now())
    raise SystemExit("--experiment-root or --experiment-id is required for resume stages")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=STAGES, default="all")
    parser.add_argument("--robot", default="xarm6")
    parser.add_argument("--config")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    parser.add_argument("--experiment-root")
    parser.add_argument("--experiment-id")
    parser.add_argument("--speeds", type=parse_float_csv, default=list(SPEED_TIERS))
    parser.add_argument("--resume", action="store_true", default=True)
    parser.add_argument("--no-resume", dest="resume", action="store_false")
    parser.add_argument("--local-count", type=int, default=100)
    parser.add_argument("--local-duration-s", type=float, default=20.0)
    parser.add_argument("--local-hz", type=float, default=100.0)
    parser.add_argument("--local-amplitude-deg", type=parse_amplitude, default=list(DEFAULT_AMPLITUDE_DEG))
    parser.add_argument("--local-accel-deg-s2", type=float, default=60.0)
    parser.add_argument("--local-hold-s", type=float, default=0.0)
    parser.add_argument("--local-waypoints", type=int, default=8)
    parser.add_argument("--local-seed-base", type=int, default=0)
    parser.add_argument("--hf-repo", action="append", dest="hf_repos")
    parser.add_argument("--hf-max-episodes", type=int)
    parser.add_argument("--hf-hz", type=float, default=100.0)
    parser.add_argument("--hf-min-frames", type=int, default=20)
    parser.add_argument("--hf-max-step-deg", type=float, default=DEFAULT_HF_MAX_STEP_DEG)
    parser.add_argument("--hf-limit-buffer-deg", type=float, default=5.0)
    parser.add_argument("--hf-start-home-tolerance-deg", type=float, default=60.0)
    parser.add_argument("--replay-acc-deg-s2", type=float, default=200.0)
    parser.add_argument("--torque-model-path")
    parser.add_argument("--dashboard-url", default="http://127.0.0.1:8765/")
    parser.add_argument("--valid-ratio", type=float, default=0.2)
    parser.add_argument("--split-seed", type=int, default=7)
    parser.add_argument("--epochs", type=int, default=200)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--hidden-dim", type=int, default=64)
    parser.add_argument("--train-seed", type=int, default=7)
    parser.add_argument("--model-output")
    parser.add_argument("--old-model", default=str(DEFAULT_OLD_MODEL))
    parser.add_argument("--new-model")
    parser.add_argument("--i-understand-real-robot-motion", action="store_true")
    parser.add_argument("--self-check", action="store_true", help="validate config/dependencies and exit without robot motion")
    return parser.parse_args(argv)


def run_stage(args: argparse.Namespace) -> Any:
    root = resolve_experiment_root(args)
    config = load_config(args.config, robot=args.robot)
    repos = tuple(args.hf_repos or DEFAULT_HF_REPOS)
    speeds = tuple(float(speed) for speed in args.speeds)
    if args.self_check:
        print(json.dumps({"root": str(root), "robot": config.get("robot_name"), "speeds": speeds, "hf_repos": repos}, indent=2, sort_keys=True))
        return root
    if args.stage in REAL_ROBOT_STAGES:
        _require_real_robot_ack(args.stage, args.i_understand_real_robot_motion)

    def generate_local():
        return generate_local_trajectories(
            config,
            root,
            speeds=speeds,
            count=args.local_count,
            duration_s=args.local_duration_s,
            hz=args.local_hz,
            amplitude_deg=args.local_amplitude_deg,
            accel_deg_s2=args.local_accel_deg_s2,
            hold_s=args.local_hold_s,
            waypoint_count=args.local_waypoints,
            seed_base=args.local_seed_base,
            resume=args.resume,
        )

    def download_hf():
        return download_hf_datasets(root, repos=repos, resume=args.resume)

    def prepare_hf():
        return prepare_hf_trajectories(
            config,
            root,
            repos=repos,
            speeds=speeds,
            hz=args.hf_hz,
            min_frames=args.hf_min_frames,
            max_step_deg=args.hf_max_step_deg,
            limit_buffer_deg=args.hf_limit_buffer_deg,
            start_home_tolerance_deg=args.hf_start_home_tolerance_deg,
            max_episodes=args.hf_max_episodes,
            resume=args.resume,
        )

    def collect_local():
        return collect_speed_ladder_torque(
            config,
            root,
            source_kind="local",
            speeds=speeds,
            replay_acc_deg_s2=args.replay_acc_deg_s2,
            model_path=args.torque_model_path,
            dashboard_url=args.dashboard_url,
            acknowledged=args.i_understand_real_robot_motion,
            resume=args.resume,
        )

    def collect_hf():
        return collect_speed_ladder_torque(
            config,
            root,
            source_kind="hf",
            speeds=speeds,
            replay_acc_deg_s2=args.replay_acc_deg_s2,
            model_path=args.torque_model_path,
            dashboard_url=args.dashboard_url,
            acknowledged=args.i_understand_real_robot_motion,
            resume=args.resume,
        )

    if args.stage == "generate-local":
        return generate_local()
    if args.stage == "download-hf":
        return download_hf()
    if args.stage == "prepare-hf":
        return prepare_hf()
    if args.stage == "collect-local":
        return collect_local()
    if args.stage == "collect-hf":
        return collect_hf()
    if args.stage == "merge":
        return merge_torque_data(root, valid_ratio=args.valid_ratio, seed=args.split_seed)
    if args.stage == "train":
        return train_speed_ladder_model(
            config,
            root,
            output_path=args.model_output,
            epochs=args.epochs,
            lr=args.lr,
            hidden_dim=args.hidden_dim,
            seed=args.train_seed,
        )
    if args.stage == "validate":
        return validate_speed_ladder_models(config, root, old_model_path=args.old_model, new_model_path=args.new_model)
    if args.stage == "all":
        generate_local()
        download_hf()
        prepare_hf()
        collect_local()
        collect_hf()
        merge_torque_data(root, valid_ratio=args.valid_ratio, seed=args.split_seed)
        model_path = train_speed_ladder_model(
            config,
            root,
            output_path=args.model_output,
            epochs=args.epochs,
            lr=args.lr,
            hidden_dim=args.hidden_dim,
            seed=args.train_seed,
        )
        return validate_speed_ladder_models(config, root, old_model_path=args.old_model, new_model_path=model_path)
    raise ValueError(f"unknown stage: {args.stage}")


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    result = run_stage(args)
    print(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
