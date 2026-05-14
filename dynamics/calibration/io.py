"""Calibration data I/O and real-time differentiation."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable
import time

import numpy as np
import pandas as pd


ARRAY_PREFIXES = ("q", "qd", "qdd", "tau_api", "tau_theory", "tau_comp", "tau_error")


class LowLatencyDifferentiator:
    """Estimate qd/qdd online with minimal smoothing and no look-ahead."""

    def __init__(self, joint_count: int, *, alpha: float = 0.65, max_abs_qdd: float | None = None):
        self.joint_count = int(joint_count)
        self.alpha = float(alpha)
        self.max_abs_qdd = max_abs_qdd
        self.prev_t: float | None = None
        self.prev_q: np.ndarray | None = None
        self.prev_qd: np.ndarray | None = None
        self.qd_filt = np.zeros(self.joint_count, dtype=np.float64)
        self.qdd_filt = np.zeros(self.joint_count, dtype=np.float64)

    def update(self, timestamp: float, q: np.ndarray, qd: np.ndarray | None = None) -> tuple[np.ndarray, np.ndarray]:
        q_arr = np.asarray(q, dtype=np.float64)
        if q_arr.shape != (self.joint_count,):
            raise ValueError(f"q must have shape ({self.joint_count},), got {q_arr.shape}")

        if self.prev_t is None or self.prev_q is None:
            qd_arr = np.zeros(self.joint_count, dtype=np.float64) if qd is None else np.asarray(qd, dtype=np.float64)
            self.prev_t = float(timestamp)
            self.prev_q = q_arr.copy()
            self.prev_qd = qd_arr.copy()
            self.qd_filt = qd_arr.copy()
            self.qdd_filt[:] = 0.0
            return self.qd_filt.copy(), self.qdd_filt.copy()

        dt = max(float(timestamp) - self.prev_t, 1e-6)
        qd_meas = np.asarray(qd, dtype=np.float64) if qd is not None else (q_arr - self.prev_q) / dt
        qdd_meas = (qd_meas - self.prev_qd) / dt if self.prev_qd is not None else np.zeros(self.joint_count)
        if self.max_abs_qdd is not None:
            qdd_meas = np.clip(qdd_meas, -self.max_abs_qdd, self.max_abs_qdd)
        a = self.alpha
        self.qd_filt = a * qd_meas + (1.0 - a) * self.qd_filt
        self.qdd_filt = a * qdd_meas + (1.0 - a) * self.qdd_filt
        self.prev_t = float(timestamp)
        self.prev_q = q_arr.copy()
        self.prev_qd = qd_meas.copy()
        return self.qd_filt.copy(), self.qdd_filt.copy()


def _put_vector(row: dict, prefix: str, values: Iterable[float] | np.ndarray | None, joint_count: int) -> None:
    if values is None:
        return
    arr = np.asarray(values, dtype=np.float64)
    if arr.shape != (joint_count,):
        raise ValueError(f"{prefix} must have shape ({joint_count},), got {arr.shape}")
    for i, value in enumerate(arr, start=1):
        row[f"{prefix}_{i}"] = float(value)


def make_record(
    *,
    timestamp: float,
    robot: str,
    joint_count: int,
    mode: str,
    q: np.ndarray,
    qd: np.ndarray | None = None,
    qdd: np.ndarray | None = None,
    tau_api: np.ndarray | None = None,
    tau_theory: np.ndarray | None = None,
    tau_comp: np.ndarray | None = None,
    tau_error: np.ndarray | None = None,
    source_file: str | None = None,
) -> dict:
    row = {
        "timestamp": float(timestamp),
        "robot": robot,
        "mode": mode,
        "source_file": source_file or "",
    }
    _put_vector(row, "q", q, joint_count)
    _put_vector(row, "qd", qd, joint_count)
    _put_vector(row, "qdd", qdd, joint_count)
    _put_vector(row, "tau_api", tau_api, joint_count)
    _put_vector(row, "tau_theory", tau_theory, joint_count)
    _put_vector(row, "tau_comp", tau_comp, joint_count)
    _put_vector(row, "tau_error", tau_error, joint_count)
    return row


def write_parquet(records: list[dict], path: str | Path) -> Path:
    if not records:
        raise ValueError("no calibration records to write")
    out = Path(path).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame.from_records(records).to_parquet(out, index=False)
    return out


def read_parquet(path: str | Path) -> pd.DataFrame:
    return pd.read_parquet(Path(path).expanduser())


def extract_matrix(df: pd.DataFrame, prefix: str, joint_count: int | None = None) -> np.ndarray:
    if joint_count is None:
        joint_count = len([col for col in df.columns if col.startswith(f"{prefix}_")])
    cols = [f"{prefix}_{i}" for i in range(1, int(joint_count) + 1)]
    missing = [col for col in cols if col not in df.columns]
    if missing:
        raise KeyError(f"missing columns for {prefix}: {missing}")
    return df[cols].to_numpy(dtype=np.float64)


def timestamped_path(directory: str | Path, robot: str, mode: str, suffix: str = ".parquet") -> Path:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return Path(directory).expanduser() / f"{robot}_{mode}_{stamp}{suffix}"


def latest_parquet(directory: str | Path) -> Path | None:
    files = sorted(Path(directory).expanduser().glob("*.parquet"), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None
