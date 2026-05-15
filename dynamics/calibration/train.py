"""Compensation model training mode."""

from __future__ import annotations

from pathlib import Path

from .compensation.hybrid import train_hybrid_compensator
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
    model_kind: str = "baseline",
    static_data_path: str | Path | None = None,
    stop_data_path: str | Path | None = None,
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
    timestamps = df["timestamp"].to_numpy(dtype=float) if "timestamp" in df else None

    if model_kind == "hybrid":
        static_kwargs = {}
        if static_data_path:
            static_df = read_parquet(static_data_path)
            static_kwargs = {
                "static_q": extract_matrix(static_df, "q", joint_count),
                "static_tau_api": extract_matrix(static_df, "tau_api", joint_count),
                "static_tau_model": extract_matrix(static_df, "tau_theory", joint_count),
            }
        stop_kwargs = {}
        if stop_data_path:
            stop_df = read_parquet(stop_data_path)
            stop_kwargs = {
                "stop_q": extract_matrix(stop_df, "q", joint_count),
                "stop_qd": extract_matrix(stop_df, "qd", joint_count),
                "stop_tau_api": extract_matrix(stop_df, "tau_api", joint_count),
                "stop_tau_model": extract_matrix(stop_df, "tau_theory", joint_count),
                "stop_timestamps": stop_df["timestamp"].to_numpy(dtype=float) if "timestamp" in stop_df else None,
            }
        bundle, metadata = train_hybrid_compensator(
            q=q,
            qd=qd,
            qdd=qdd,
            tau_api=tau_api,
            tau_model=tau_theory,
            timestamps=timestamps,
            epochs=epochs,
            lr=lr,
            hidden_dim=hidden_dim,
            **static_kwargs,
            **stop_kwargs,
        )
        path = bundle.save(output_path)
        print(f"[TRAIN] saved hybrid compensation model to {path}; metadata={metadata}")
        return path

    if model_kind != "baseline":
        raise ValueError("model_kind must be baseline or hybrid")

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
