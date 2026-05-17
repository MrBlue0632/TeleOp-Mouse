"""Compensation model training mode."""

from __future__ import annotations

from pathlib import Path

from dynamics.embodiment import build_embodiment_metadata
from dynamics.resolver import resolve_robot

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


def extract_tau_model(df, joint_count: int):
    """Read unified tau_model columns, falling back to legacy tau_theory."""
    try:
        return extract_matrix(df, "tau_model", joint_count)
    except KeyError:
        return extract_matrix(df, "tau_theory", joint_count)


def _embodiment_from_config(config: dict):
    if "urdf_path" not in config:
        return None
    robot = resolve_robot(
        config["urdf_path"],
        name=str(config.get("robot_name", "robot")),
        payload=config.get("payload"),
    )
    return build_embodiment_metadata(config, robot)


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
    tau_theory = extract_tau_model(df, joint_count)
    timestamps = df["timestamp"].to_numpy(dtype=float) if "timestamp" in df else None
    embodiment = _embodiment_from_config(config)

    if model_kind == "hybrid":
        static_kwargs = {}
        if static_data_path:
            static_df = read_parquet(static_data_path)
            static_kwargs = {
                "static_q": extract_matrix(static_df, "q", joint_count),
                "static_tau_api": extract_matrix(static_df, "tau_api", joint_count),
                "static_tau_model": extract_tau_model(static_df, joint_count),
            }
        stop_kwargs = {}
        if stop_data_path:
            stop_df = read_parquet(stop_data_path)
            stop_kwargs = {
                "stop_q": extract_matrix(stop_df, "q", joint_count),
                "stop_qd": extract_matrix(stop_df, "qd", joint_count),
                "stop_tau_api": extract_matrix(stop_df, "tau_api", joint_count),
                "stop_tau_model": extract_tau_model(stop_df, joint_count),
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
            embodiment=embodiment,
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
        extra={
            "source_file": str(torque_file),
            "final_loss": losses[-1] if losses else None,
            **({"embodiment": embodiment} if embodiment is not None else {}),
        },
    )
    print(f"[TRAIN] saved compensation model to {path}; final_loss={losses[-1]:.6f}")
    return path
