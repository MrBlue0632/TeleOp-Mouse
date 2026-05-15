"""Hybrid torque compensation model.

The hybrid model decomposes xArm torque API residuals into three parts:
static position bias, motion-related residuals, and firmware/current state
bias after motion stops.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch

from .firmware_state import (
    FirmwareBiasTracker,
    FirmwareStateModel,
    derive_motion_lambda,
    derive_time_since_stop,
)
from .per_joint_mlp import PerJointMotionModel, train_per_joint_motion_model
from .static_bias import StaticBiasModel


@dataclass
class HybridTorqueEstimate:
    tau_external: np.ndarray
    tau_static_bias: np.ndarray
    tau_motion_comp: np.ndarray
    tau_firmware_bias: np.ndarray
    tau_comp: np.ndarray
    motion_lambda: float
    time_since_stop: np.ndarray
    firmware_state: np.ndarray
    is_moving: np.ndarray


class HybridTorqueCompensator:
    """Runtime hybrid compensator with stateful firmware-bias tracking."""

    kind = "hybrid"

    def __init__(
        self,
        *,
        static_model: StaticBiasModel,
        firmware_model: FirmwareStateModel,
        motion_model: PerJointMotionModel,
        metadata: dict[str, Any] | None = None,
    ):
        self.static_model = static_model
        self.firmware_model = firmware_model
        self.motion_model = motion_model
        self.metadata = metadata or {}
        self.tracker = FirmwareBiasTracker(firmware_model)

    @property
    def joint_count(self) -> int:
        return self.static_model.joint_count

    def reset(self) -> None:
        self.tracker.reset()

    def update(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        qdd: np.ndarray,
        tau_api: np.ndarray,
        tau_model: np.ndarray,
        timestamp: float | None = None,
    ) -> HybridTorqueEstimate:
        q_arr = np.asarray(q, dtype=np.float64)[: self.joint_count]
        qd_arr = np.asarray(qd, dtype=np.float64)[: self.joint_count]
        qdd_arr = np.asarray(qdd, dtype=np.float64)[: self.joint_count]
        tau_api_arr = np.asarray(tau_api, dtype=np.float64)[: self.joint_count]
        tau_model_arr = np.asarray(tau_model, dtype=np.float64)[: self.joint_count]

        tau_static = self.static_model.predict(q_arr)
        residual_after_static = tau_api_arr - tau_model_arr - tau_static
        fw = self.tracker.update(qd_arr, residual_after_static, timestamp=timestamp)
        tau_motion = self.motion_model.predict(
            q_arr,
            qd_arr,
            qdd_arr,
            tau_model_arr,
            fw.motion_lambda,
            fw.time_since_stop,
        )
        tau_comp = tau_static + fw.bias + tau_motion
        tau_external = tau_api_arr - tau_model_arr - tau_comp
        return HybridTorqueEstimate(
            tau_external=tau_external,
            tau_static_bias=tau_static,
            tau_motion_comp=tau_motion,
            tau_firmware_bias=fw.bias,
            tau_comp=tau_comp,
            motion_lambda=fw.motion_lambda,
            time_since_stop=fw.time_since_stop,
            firmware_state=fw.detected_levels,
            is_moving=fw.is_moving,
        )

    def predict_compensation(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        qdd: np.ndarray,
        tau_theory: np.ndarray | None = None,
    ) -> np.ndarray:
        """Stateless-ish compatibility API used by the old baseline caller."""
        tau_model = np.zeros(self.joint_count, dtype=np.float64) if tau_theory is None else np.asarray(tau_theory, dtype=np.float64)
        tau_static = self.static_model.predict(q)
        zeros = np.zeros(self.joint_count, dtype=np.float64)
        tau_motion = self.motion_model.predict(q, qd, qdd, tau_model, 0.0, zeros)
        return tau_static + tau_motion

    def checkpoint(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "static_model": self.static_model.to_dict(),
            "firmware_model": self.firmware_model.to_dict(),
            "motion_model": self.motion_model.to_checkpoint(),
            "metadata": self.metadata,
        }

    def save(self, path: str | Path) -> Path:
        out = Path(path).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        torch.save(self.checkpoint(), out)
        return out

    @classmethod
    def from_checkpoint(cls, checkpoint: dict[str, Any]) -> "HybridTorqueCompensator":
        return cls(
            static_model=StaticBiasModel.from_dict(checkpoint["static_model"]),
            firmware_model=FirmwareStateModel.from_dict(checkpoint["firmware_model"]),
            motion_model=PerJointMotionModel.from_checkpoint(checkpoint["motion_model"]),
            metadata=dict(checkpoint.get("metadata", {})),
        )

    @classmethod
    def load(cls, path: str | Path) -> "HybridTorqueCompensator":
        checkpoint = torch.load(Path(path).expanduser(), map_location="cpu", weights_only=False)
        if not isinstance(checkpoint, dict) or checkpoint.get("kind") != cls.kind:
            raise ValueError(f"not a hybrid compensation checkpoint: {path}")
        return cls.from_checkpoint(checkpoint)


def train_hybrid_compensator(
    *,
    q: np.ndarray,
    qd: np.ndarray,
    qdd: np.ndarray,
    tau_api: np.ndarray,
    tau_model: np.ndarray,
    timestamps: np.ndarray | None = None,
    static_q: np.ndarray | None = None,
    static_tau_api: np.ndarray | None = None,
    static_tau_model: np.ndarray | None = None,
    stop_q: np.ndarray | None = None,
    stop_qd: np.ndarray | None = None,
    stop_residual: np.ndarray | None = None,
    stop_tau_api: np.ndarray | None = None,
    stop_tau_model: np.ndarray | None = None,
    stop_timestamps: np.ndarray | None = None,
    static_alpha: float = 1.0,
    speed_threshold: float = np.deg2rad(2.0),
    epochs: int = 200,
    lr: float = 1e-3,
    hidden_dim: int = 64,
    seed: int = 7,
) -> tuple[HybridTorqueCompensator, dict[str, Any]]:
    q_arr = np.asarray(q, dtype=np.float64)
    qd_arr = np.asarray(qd, dtype=np.float64)
    qdd_arr = np.asarray(qdd, dtype=np.float64)
    tau_api_arr = np.asarray(tau_api, dtype=np.float64)
    tau_model_arr = np.asarray(tau_model, dtype=np.float64)
    if q_arr.ndim != 2 or not (q_arr.shape == qd_arr.shape == qdd_arr.shape == tau_api_arr.shape == tau_model_arr.shape):
        raise ValueError("q, qd, qdd, tau_api, and tau_model must all have shape (N, J)")
    joint_count = int(q_arr.shape[1])

    residual = tau_api_arr - tau_model_arr
    if static_q is None or static_tau_api is None or static_tau_model is None:
        static_mask = np.max(np.abs(qd_arr), axis=1) <= float(speed_threshold)
        if np.count_nonzero(static_mask) < max(4, joint_count):
            static_model = StaticBiasModel.zeros(joint_count)
        else:
            static_model = StaticBiasModel.fit(q_arr[static_mask], residual[static_mask], alpha=static_alpha)
    else:
        static_residual = np.asarray(static_tau_api, dtype=np.float64) - np.asarray(static_tau_model, dtype=np.float64)
        static_model = StaticBiasModel.fit(np.asarray(static_q, dtype=np.float64), static_residual, alpha=static_alpha)

    residual_after_static = residual - static_model.predict(q_arr)
    if stop_residual is None and stop_q is not None and stop_tau_api is not None and stop_tau_model is not None:
        stop_residual = (
            np.asarray(stop_tau_api, dtype=np.float64)
            - np.asarray(stop_tau_model, dtype=np.float64)
            - static_model.predict(np.asarray(stop_q, dtype=np.float64))
        )

    if stop_qd is None or stop_residual is None:
        firmware_model = FirmwareStateModel.fit(
            residual_after_static,
            qd_arr,
            timestamps,
            speed_threshold=speed_threshold,
        )
    else:
        firmware_model = FirmwareStateModel.fit(
            np.asarray(stop_residual, dtype=np.float64),
            np.asarray(stop_qd, dtype=np.float64),
            stop_timestamps,
            speed_threshold=speed_threshold,
        )

    time_since_stop = derive_time_since_stop(qd_arr, timestamps, firmware_model.speed_threshold)
    motion_lambda = derive_motion_lambda(
        qd_arr,
        speed_threshold=firmware_model.speed_threshold,
        blend_alpha=firmware_model.blend_alpha,
    )
    firmware_bias = estimate_firmware_bias_sequence(firmware_model, qd_arr, residual_after_static, timestamps)
    motion_target = residual_after_static - firmware_bias
    motion_model, losses = train_per_joint_motion_model(
        q_arr,
        qd_arr,
        qdd_arr,
        tau_model_arr,
        motion_lambda,
        time_since_stop,
        motion_target,
        epochs=epochs,
        lr=lr,
        hidden_dim=hidden_dim,
        seed=seed,
    )

    final_losses = [joint_losses[-1] if joint_losses else None for joint_losses in losses]
    metadata = {
        "joint_count": joint_count,
        "speed_threshold": firmware_model.speed_threshold,
        "static_rmse": static_model.residual_rmse.astype(float).tolist(),
        "motion_final_losses": final_losses,
    }
    return (
        HybridTorqueCompensator(
            static_model=static_model,
            firmware_model=firmware_model,
            motion_model=motion_model,
            metadata=metadata,
        ),
        metadata,
    )


def estimate_firmware_bias_sequence(
    firmware_model: FirmwareStateModel,
    qd: np.ndarray,
    residual_after_static: np.ndarray,
    timestamps: np.ndarray | None,
) -> np.ndarray:
    tracker = FirmwareBiasTracker(firmware_model)
    qd_arr = np.asarray(qd, dtype=np.float64)
    residual_arr = np.asarray(residual_after_static, dtype=np.float64)
    if timestamps is None:
        ts = np.arange(qd_arr.shape[0], dtype=np.float64) * 0.01
    else:
        ts = np.asarray(timestamps, dtype=np.float64)
    out = np.zeros_like(residual_arr)
    for idx in range(qd_arr.shape[0]):
        out[idx] = tracker.update(qd_arr[idx], residual_arr[idx], timestamp=float(ts[idx])).bias
    return out
