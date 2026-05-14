"""Compensation model training mode."""

from __future__ import annotations

from pathlib import Path

from .compensation.mlp import save_compensation, train_compensation
from .io import extract_matrix, latest_parquet, read_parquet


def resolve_torque_path(path: str | Path | None, torque_dir: str | Path = "dynamics/calibration/torque") -> Path:
    if path:
        out = Path(path).expanduser()
    else:
        found = latest_parquet(torque_dir)
        if found is None:
            raise FileNotFoundError(f"no torque parquet found in {torque_dir}; run --mode torque first")
        out = found
    if not out.exists():
        raise FileNotFoundError(f"torque file not found: {out}")
    return out


def train_from_torque_data(
    config: dict,
    *,
    data_path: str | Path | None = None,
    output_path: str | Path = "dynamics/calibration/compensation/compensation.pt",
    target: str = "residual",
    epochs: int = 200,
    lr: float = 1e-3,
    hidden_dim: int = 64,
) -> Path:
    joint_count = int(config.get("joint_count", 6))
    torque_file = resolve_torque_path(data_path)
    df = read_parquet(torque_file)
    q = extract_matrix(df, "q", joint_count)
    qd = extract_matrix(df, "qd", joint_count)
    qdd = extract_matrix(df, "qdd", joint_count)
    tau_api = extract_matrix(df, "tau_api", joint_count)
    tau_theory = extract_matrix(df, "tau_theory", joint_count)
    bundle, losses = train_compensation(
        q,
        qd,
        qdd,
        tau_api,
        tau_theory,
        target=target,
        epochs=epochs,
        lr=lr,
        hidden_dim=hidden_dim,
    )
    path = save_compensation(
        bundle,
        output_path,
        extra={"source_file": str(torque_file), "final_loss": losses[-1] if losses else None},
    )
    print(f"[TRAIN] saved compensation model to {path}; final_loss={losses[-1]:.6f}")
    return path
