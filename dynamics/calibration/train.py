"""Compensation model training mode."""

from __future__ import annotations

import math
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


def _section(config: dict, name: str) -> dict:
    value = config.get(name, {})
    return value if isinstance(value, dict) else {}


def _coalesce(*values):
    for value in values:
        if value is not None:
            return value
    return None


def train_from_torque_data(
    config: dict,
    *,
    data_path: str | Path | None = None,
    output_path: str | Path | None = None,
    model_kind: str | None = None,
    static_data_path: str | Path | None = None,
    stop_data_path: str | Path | None = None,
    target: str | None = None,
    epochs: int | None = None,
    lr: float | None = None,
    hidden_dim: int | None = None,
    seed: int | None = None,
    static_alpha: float | None = None,
    speed_threshold_deg_s: float | None = None,
    motion_history_steps: int | None = None,
) -> Path:
    paths = _section(config, "paths")
    training_cfg = _section(config, "training")
    compensation_cfg = _section(config, "compensation")
    hybrid_cfg = _section(compensation_cfg, "hybrid")

    data_path = _coalesce(data_path, training_cfg.get("data_path"))
    output_path = _coalesce(output_path, paths.get("compensation_model"), "dynamics/calibration/compensation/compensation.pt")
    model_kind = str(_coalesce(model_kind, training_cfg.get("model_kind"), "baseline"))
    static_data_path = _coalesce(static_data_path, training_cfg.get("static_data_path"))
    stop_data_path = _coalesce(stop_data_path, training_cfg.get("stop_data_path"))
    target = str(_coalesce(target, training_cfg.get("target"), "residual"))
    epochs = int(_coalesce(epochs, training_cfg.get("epochs"), 200))
    lr = float(_coalesce(lr, training_cfg.get("lr"), 1e-3))
    hidden_dim = int(_coalesce(hidden_dim, training_cfg.get("hidden_dim"), 64))
    seed = int(_coalesce(seed, training_cfg.get("seed"), 7))
    static_alpha = float(_coalesce(static_alpha, hybrid_cfg.get("static_alpha"), 1.0))
    speed_threshold_deg_s = float(_coalesce(speed_threshold_deg_s, hybrid_cfg.get("speed_threshold_deg_s"), 2.0))
    motion_history_steps = int(_coalesce(motion_history_steps, hybrid_cfg.get("motion_history_steps"), 3))
    firmware_min_level_gap = float(_coalesce(hybrid_cfg.get("min_level_gap_nm"), 1.0))
    firmware_default_decay_tau_s = float(_coalesce(hybrid_cfg.get("default_decay_tau_s"), 0.35))
    firmware_blend_alpha = float(_coalesce(hybrid_cfg.get("blend_alpha"), 0.2))
    firmware_settle_lambda_threshold = float(_coalesce(hybrid_cfg.get("settle_lambda_threshold"), 0.05))
    firmware_detect_ema_alpha = float(_coalesce(hybrid_cfg.get("detect_ema_alpha"), 0.05))
    firmware_j3_jump_size = float(_coalesce(hybrid_cfg.get("j3_jump_size_nm"), 5.28))

    joint_count = int(config.get("joint_count", 6))
    torque_file = resolve_torque_path(data_path, torque_dir=paths.get("torque_dir", "dynamics/calibration/torque"))
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
            seed=seed,
            static_alpha=static_alpha,
            speed_threshold=math.radians(speed_threshold_deg_s),
            history_steps=motion_history_steps,
            firmware_min_level_gap=firmware_min_level_gap,
            firmware_default_decay_tau_s=firmware_default_decay_tau_s,
            firmware_blend_alpha=firmware_blend_alpha,
            firmware_settle_lambda_threshold=firmware_settle_lambda_threshold,
            firmware_detect_ema_alpha=firmware_detect_ema_alpha,
            firmware_j3_jump_size=firmware_j3_jump_size,
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
        seed=seed,
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
