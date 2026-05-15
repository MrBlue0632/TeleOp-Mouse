"""Independent per-joint MLPs for motion residual compensation."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np
import torch
from torch import nn


class JointMLP(nn.Module):
    def __init__(self, input_dim: int, hidden_dim: int = 64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, 1),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x).squeeze(-1)


def _as_matrix(values: np.ndarray, joint_count: int, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr[None, :]
    if arr.ndim != 2 or arr.shape[1] != joint_count:
        raise ValueError(f"{name} must have shape (N, {joint_count}), got {arr.shape}")
    return arr


def motion_features(
    q: np.ndarray,
    qd: np.ndarray,
    qdd: np.ndarray,
    tau_model: np.ndarray,
    motion_lambda: np.ndarray,
    time_since_stop: np.ndarray,
    joint_index: int,
) -> np.ndarray:
    """Build motion residual features for one joint.

    q, qd, and qdd are expected to use the same units as the training data.
    In the calibration pipeline this is radians / radians per second.
    """
    q_arr = np.asarray(q, dtype=np.float64)
    if q_arr.ndim == 1:
        q_arr = q_arr[None, :]
    joint_count = int(q_arr.shape[1])
    qd_arr = _as_matrix(qd, joint_count, "qd")
    qdd_arr = _as_matrix(qdd, joint_count, "qdd")
    tau_model_arr = _as_matrix(tau_model, joint_count, "tau_model")
    tss_arr = _as_matrix(time_since_stop, joint_count, "time_since_stop")
    lam = np.asarray(motion_lambda, dtype=np.float64)
    if lam.ndim == 0:
        lam = np.full(q_arr.shape[0], float(lam), dtype=np.float64)
    if lam.shape != (q_arr.shape[0],):
        raise ValueError(f"motion_lambda must have shape ({q_arr.shape[0]},), got {lam.shape}")

    j = int(joint_index)
    parts = [
        np.sin(q_arr),
        np.cos(q_arr),
        qd_arr,
        np.abs(qd_arr),
        qdd_arr,
        np.abs(qdd_arr),
        tau_model_arr[:, [j]],
        lam[:, None],
        tss_arr[:, [j]],
        np.sign(qd_arr[:, [j]]),
    ]
    return np.concatenate(parts, axis=1).astype(np.float32)


@dataclass
class PerJointMotionModel:
    models: list[JointMLP]
    x_mean: list[np.ndarray]
    x_std: list[np.ndarray]
    y_mean: np.ndarray
    y_std: np.ndarray
    joint_count: int
    hidden_dim: int = 64

    @classmethod
    def zeros(cls, joint_count: int, *, input_dim: int | None = None, hidden_dim: int = 64) -> "PerJointMotionModel":
        if input_dim is None:
            sample = np.zeros((1, joint_count), dtype=np.float64)
            input_dim = int(motion_features(sample, sample, sample, sample, np.zeros(1), sample, 0).shape[1])
        models = [JointMLP(input_dim, hidden_dim=hidden_dim) for _ in range(joint_count)]
        return cls(
            models=models,
            x_mean=[np.zeros(input_dim, dtype=np.float32) for _ in range(joint_count)],
            x_std=[np.ones(input_dim, dtype=np.float32) for _ in range(joint_count)],
            y_mean=np.zeros(joint_count, dtype=np.float32),
            y_std=np.ones(joint_count, dtype=np.float32),
            joint_count=int(joint_count),
            hidden_dim=int(hidden_dim),
        )

    def predict(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        qdd: np.ndarray,
        tau_model: np.ndarray,
        motion_lambda: np.ndarray | float,
        time_since_stop: np.ndarray,
    ) -> np.ndarray:
        q_arr = np.asarray(q, dtype=np.float64)
        single = q_arr.ndim == 1
        if single:
            q_arr = q_arr[None, :]
        qd_arr = _as_matrix(qd, self.joint_count, "qd")
        qdd_arr = _as_matrix(qdd, self.joint_count, "qdd")
        tau_arr = _as_matrix(tau_model, self.joint_count, "tau_model")
        tss_arr = _as_matrix(time_since_stop, self.joint_count, "time_since_stop")
        lam = np.asarray(motion_lambda, dtype=np.float64)
        if lam.ndim == 0:
            lam = np.full(q_arr.shape[0], float(lam), dtype=np.float64)

        out = np.zeros((q_arr.shape[0], self.joint_count), dtype=np.float64)
        for joint, model in enumerate(self.models):
            x = motion_features(q_arr, qd_arr, qdd_arr, tau_arr, lam, tss_arr, joint)
            x_norm = (x - self.x_mean[joint]) / self.x_std[joint]
            model.eval()
            with torch.no_grad():
                y_norm = model(torch.from_numpy(x_norm.astype(np.float32))).cpu().numpy()
            out[:, joint] = y_norm * self.y_std[joint] + self.y_mean[joint]
        return out[0] if single else out

    def to_checkpoint(self) -> dict[str, Any]:
        return {
            "state_dicts": [model.state_dict() for model in self.models],
            "x_mean": [x.astype(np.float32) for x in self.x_mean],
            "x_std": [x.astype(np.float32) for x in self.x_std],
            "y_mean": self.y_mean.astype(np.float32),
            "y_std": self.y_std.astype(np.float32),
            "joint_count": self.joint_count,
            "hidden_dim": self.hidden_dim,
            "input_dim": int(self.x_mean[0].shape[0]) if self.x_mean else 0,
        }

    @classmethod
    def from_checkpoint(cls, data: dict[str, Any]) -> "PerJointMotionModel":
        joint_count = int(data["joint_count"])
        hidden_dim = int(data.get("hidden_dim", 64))
        input_dim = int(data["input_dim"])
        models = [JointMLP(input_dim, hidden_dim=hidden_dim) for _ in range(joint_count)]
        for model, state_dict in zip(models, data["state_dicts"]):
            model.load_state_dict(state_dict)
        return cls(
            models=models,
            x_mean=[np.asarray(x, dtype=np.float32) for x in data["x_mean"]],
            x_std=[np.asarray(x, dtype=np.float32) for x in data["x_std"]],
            y_mean=np.asarray(data["y_mean"], dtype=np.float32),
            y_std=np.asarray(data["y_std"], dtype=np.float32),
            joint_count=joint_count,
            hidden_dim=hidden_dim,
        )


def train_per_joint_motion_model(
    q: np.ndarray,
    qd: np.ndarray,
    qdd: np.ndarray,
    tau_model: np.ndarray,
    motion_lambda: np.ndarray,
    time_since_stop: np.ndarray,
    target: np.ndarray,
    *,
    epochs: int = 200,
    lr: float = 1e-3,
    hidden_dim: int = 64,
    seed: int = 7,
) -> tuple[PerJointMotionModel, list[list[float]]]:
    q_arr = np.asarray(q, dtype=np.float64)
    if q_arr.ndim != 2:
        raise ValueError(f"q must have shape (N, J), got {q_arr.shape}")
    joint_count = int(q_arr.shape[1])
    target_arr = _as_matrix(target, joint_count, "target")
    torch.manual_seed(seed)

    models: list[JointMLP] = []
    x_means: list[np.ndarray] = []
    x_stds: list[np.ndarray] = []
    y_mean = np.zeros(joint_count, dtype=np.float32)
    y_std = np.ones(joint_count, dtype=np.float32)
    losses_by_joint: list[list[float]] = []

    for joint in range(joint_count):
        x = motion_features(q, qd, qdd, tau_model, motion_lambda, time_since_stop, joint)
        y = target_arr[:, joint].astype(np.float32)
        x_mean = x.mean(axis=0).astype(np.float32)
        x_std = (x.std(axis=0) + 1e-6).astype(np.float32)
        y_mean[joint] = float(y.mean())
        y_std[joint] = float(y.std() + 1e-6)
        x_norm = (x - x_mean) / x_std
        y_norm = (y - y_mean[joint]) / y_std[joint]

        model = JointMLP(int(x.shape[1]), hidden_dim=hidden_dim)
        opt = torch.optim.AdamW(model.parameters(), lr=lr)
        loss_fn = nn.MSELoss()
        xt = torch.from_numpy(x_norm.astype(np.float32))
        yt = torch.from_numpy(y_norm.astype(np.float32))
        losses: list[float] = []
        for _ in range(int(epochs)):
            opt.zero_grad(set_to_none=True)
            loss = loss_fn(model(xt), yt)
            loss.backward()
            opt.step()
            losses.append(float(loss.detach().cpu().item()))

        models.append(model)
        x_means.append(x_mean)
        x_stds.append(x_std)
        losses_by_joint.append(losses)

    return (
        PerJointMotionModel(
            models=models,
            x_mean=x_means,
            x_std=x_stds,
            y_mean=y_mean,
            y_std=y_std,
            joint_count=joint_count,
            hidden_dim=int(hidden_dim),
        ),
        losses_by_joint,
    )
