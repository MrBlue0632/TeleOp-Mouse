"""Offline force-estimation metrics for no-contact xArm trajectories."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Sequence

import numpy as np
import pandas as pd

from .io import extract_matrix, read_parquet


DEFAULT_JOINT_COUNT = 6
DEFAULT_SPEED_THRESHOLD_DEG_S = 2.0
DEFAULT_STOP_WINDOW_S = 1.0


def _stack_vector_column(df: pd.DataFrame, column: str, joint_count: int) -> np.ndarray:
    values = []
    for value in df[column].to_numpy():
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
        if arr.size < joint_count:
            arr = np.pad(arr, (0, joint_count - arr.size), constant_values=np.nan)
        values.append(arr[:joint_count])
    return np.vstack(values).astype(np.float64)


def _matrix_from_any(df: pd.DataFrame, *, vector_column: str, prefix: str, joint_count: int) -> np.ndarray | None:
    if vector_column in df.columns:
        return _stack_vector_column(df, vector_column, joint_count)
    cols = [f"{prefix}_{idx}" for idx in range(1, joint_count + 1)]
    if all(col in df.columns for col in cols):
        return extract_matrix(df, prefix, joint_count)
    return None


def _action_matrix(df: pd.DataFrame) -> np.ndarray | None:
    if "action" not in df.columns:
        return None
    values = []
    for value in df["action"].to_numpy():
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
        if arr.size < 6:
            arr = np.pad(arr, (0, 6 - arr.size), constant_values=0.0)
        values.append(arr[:6])
    return np.vstack(values).astype(np.float64)


def _timestamps(df: pd.DataFrame) -> np.ndarray:
    if "timestamp" in df.columns:
        ts = df["timestamp"].to_numpy(dtype=np.float64)
    elif "frame_index" in df.columns:
        ts = df["frame_index"].to_numpy(dtype=np.float64) / 30.0
    else:
        ts = np.arange(len(df), dtype=np.float64) / 30.0
    if ts.size == 0:
        return ts
    return ts - float(ts[0])


def _read_dataset_frames(path: Path) -> pd.DataFrame:
    if path.is_file():
        return read_parquet(path)
    data_files = sorted((path / "data").glob("chunk-*/file-*.parquet"))
    if not data_files:
        data_files = sorted(path.glob("*.parquet"))
    if not data_files:
        raise FileNotFoundError(f"no parquet data files found under {path}")
    return pd.concat([read_parquet(file) for file in data_files], ignore_index=True)


def derive_motion_masks(
    df: pd.DataFrame,
    *,
    joint_count: int = DEFAULT_JOINT_COUNT,
    speed_threshold_deg_s: float = DEFAULT_SPEED_THRESHOLD_DEG_S,
    stop_window_s: float = DEFAULT_STOP_WINDOW_S,
) -> dict[str, np.ndarray]:
    """Return all/motion/static/stop-window boolean masks for one dataframe."""
    n = len(df)
    all_mask = np.ones(n, dtype=bool)
    action = _action_matrix(df)
    if action is not None:
        moving = np.linalg.norm(action, axis=1) > 1e-6
    else:
        qd = _matrix_from_any(df, vector_column="observation.joint_velocities_deg_s", prefix="qd", joint_count=joint_count)
        if qd is None:
            moving = np.zeros(n, dtype=bool)
        else:
            threshold = float(speed_threshold_deg_s)
            if "observation.joint_velocities_deg_s" not in df.columns:
                threshold = float(np.deg2rad(speed_threshold_deg_s))
            moving = np.nanmax(np.abs(qd), axis=1) > threshold

    ts = _timestamps(df)
    stop_window = np.zeros(n, dtype=bool)
    stop_start: float | None = None
    prev_moving = False
    for idx, is_moving in enumerate(moving):
        if is_moving:
            stop_start = None
        else:
            if prev_moving:
                stop_start = float(ts[idx])
            if stop_start is not None and 0.0 <= float(ts[idx]) - stop_start <= float(stop_window_s):
                stop_window[idx] = True
        prev_moving = bool(is_moving)

    return {
        "all": all_mask,
        "motion": moving,
        "static": ~moving,
        "stop_0_1s": stop_window,
    }


def _nan_block(joint_count: int) -> dict[str, Any]:
    return {
        "frames": 0,
        "rms_joints_nm": [float("nan")] * joint_count,
        "rms_mean_nm": float("nan"),
        "abs_p95_joints_nm": [float("nan")] * joint_count,
        "abs_p95_mean_nm": float("nan"),
        "norm_p95_nm": float("nan"),
        "norm_max_nm": float("nan"),
        "abs_gt2_pct": float("nan"),
        "abs_gt5_pct": float("nan"),
        "norm_gt10_pct": float("nan"),
    }


def metric_block(tau_external: np.ndarray, mask: np.ndarray, *, joint_count: int = DEFAULT_JOINT_COUNT) -> dict[str, Any]:
    values = np.asarray(tau_external, dtype=np.float64)[mask]
    if values.size == 0:
        return _nan_block(joint_count)
    abs_values = np.abs(values)
    norm = np.linalg.norm(values, axis=1)
    rms = np.sqrt(np.nanmean(values * values, axis=0))
    p95 = np.nanpercentile(abs_values, 95, axis=0)
    return {
        "frames": int(values.shape[0]),
        "rms_joints_nm": np.round(rms, 3).astype(float).tolist(),
        "rms_mean_nm": round(float(np.nanmean(rms)), 3),
        "abs_p95_joints_nm": np.round(p95, 3).astype(float).tolist(),
        "abs_p95_mean_nm": round(float(np.nanmean(p95)), 3),
        "norm_p95_nm": round(float(np.nanpercentile(norm, 95)), 3),
        "norm_max_nm": round(float(np.nanmax(norm)), 3),
        "abs_gt2_pct": round(float(np.nanmean(abs_values > 2.0) * 100.0), 3),
        "abs_gt5_pct": round(float(np.nanmean(abs_values > 5.0) * 100.0), 3),
        "norm_gt10_pct": round(float(np.nanmean(norm > 10.0) * 100.0), 3),
    }


def evaluate_force_estimation_path(
    path: str | Path,
    *,
    joint_count: int = DEFAULT_JOINT_COUNT,
    speed_threshold_deg_s: float = DEFAULT_SPEED_THRESHOLD_DEG_S,
    stop_window_s: float = DEFAULT_STOP_WINDOW_S,
) -> dict[str, Any]:
    """Evaluate one LeRobot dataset directory or calibration parquet file."""
    source = Path(path).expanduser()
    df = _read_dataset_frames(source)
    tau_external = _matrix_from_any(df, vector_column="observation.torque_external", prefix="tau_external", joint_count=joint_count)
    if tau_external is None:
        raise KeyError(f"{source} does not contain observation.torque_external or tau_external_i columns")
    masks = derive_motion_masks(
        df,
        joint_count=joint_count,
        speed_threshold_deg_s=speed_threshold_deg_s,
        stop_window_s=stop_window_s,
    )
    metrics = {name: metric_block(tau_external, mask, joint_count=joint_count) for name, mask in masks.items()}
    ts = _timestamps(df)
    return {
        "path": str(source),
        "name": source.name,
        "rows": int(len(df)),
        "duration_s": round(float(ts[-1] - ts[0]), 3) if ts.size > 1 else 0.0,
        "metrics": metrics,
    }


def evaluate_many(paths: Sequence[str | Path], **kwargs: Any) -> list[dict[str, Any]]:
    return [evaluate_force_estimation_path(path, **kwargs) for path in paths]


def format_report(results: Sequence[dict[str, Any]]) -> str:
    lines = [
        "dataset,rows,duration_s,all_rms_mean,all_norm_p95,all_norm_max,stop_rms_mean,stop_norm_p95",
    ]
    for result in results:
        all_metrics = result["metrics"]["all"]
        stop_metrics = result["metrics"]["stop_0_1s"]
        lines.append(
            ",".join(
                [
                    str(result["name"]),
                    str(result["rows"]),
                    f"{result['duration_s']:.3f}",
                    f"{all_metrics['rms_mean_nm']:.3f}",
                    f"{all_metrics['norm_p95_nm']:.3f}",
                    f"{all_metrics['norm_max_nm']:.3f}",
                    f"{stop_metrics['rms_mean_nm']:.3f}",
                    f"{stop_metrics['norm_p95_nm']:.3f}",
                ]
            )
        )
    return "\n".join(lines)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate no-contact force-estimation residuals")
    parser.add_argument("paths", nargs="+", help="LeRobot dataset directories or torque parquet files")
    parser.add_argument("--joint-count", type=int, default=DEFAULT_JOINT_COUNT)
    parser.add_argument("--speed-threshold-deg-s", type=float, default=DEFAULT_SPEED_THRESHOLD_DEG_S)
    parser.add_argument("--stop-window-s", type=float, default=DEFAULT_STOP_WINDOW_S)
    parser.add_argument("--json", action="store_true", help="Print full JSON metrics")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    results = evaluate_many(
        args.paths,
        joint_count=args.joint_count,
        speed_threshold_deg_s=args.speed_threshold_deg_s,
        stop_window_s=args.stop_window_s,
    )
    if args.json:
        print(json.dumps(results, indent=2, sort_keys=True))
    else:
        print(format_report(results))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
