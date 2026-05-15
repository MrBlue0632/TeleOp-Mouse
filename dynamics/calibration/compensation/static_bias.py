"""Position-dependent static torque bias model."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np


def trig_features(q: np.ndarray, joint_count: int | None = None) -> np.ndarray:
    """Build compact trigonometric features for static gravity residuals.

    Inputs are in radians.  The coupled terms follow the xArm6 vertical-chain
    joints but are guarded so the helper remains usable in small unit tests.
    """
    q_arr = np.asarray(q, dtype=np.float64)
    if q_arr.ndim != 1:
        raise ValueError(f"q must be 1-D, got {q_arr.shape}")
    if joint_count is None:
        joint_count = int(q_arr.shape[0])
    q_arr = q_arr[:joint_count]

    feats: list[float] = [1.0]
    for value in q_arr:
        feats.extend([float(np.sin(value)), float(np.cos(value))])

    coupled_indices = [(1, 2), (1, 2, 3), (1, 2, 4)]
    for indices in coupled_indices:
        if all(idx < joint_count for idx in indices):
            total = float(np.sum(q_arr[list(indices)]))
            feats.extend([float(np.sin(total)), float(np.cos(total))])

    return np.asarray(feats, dtype=np.float64)


def build_feature_matrix(q: np.ndarray, joint_count: int | None = None) -> np.ndarray:
    q_arr = np.asarray(q, dtype=np.float64)
    if q_arr.ndim != 2:
        raise ValueError(f"q must have shape (N, J), got {q_arr.shape}")
    return np.vstack([trig_features(row, joint_count) for row in q_arr])


@dataclass
class StaticBiasModel:
    """Per-joint ridge model for residual torque at settled static poses."""

    coefficients: np.ndarray
    residual_rmse: np.ndarray
    joint_count: int
    alpha: float = 1.0

    @classmethod
    def fit(cls, q: np.ndarray, residual: np.ndarray, *, alpha: float = 1.0) -> "StaticBiasModel":
        q_arr = np.asarray(q, dtype=np.float64)
        y = np.asarray(residual, dtype=np.float64)
        if q_arr.ndim != 2 or y.ndim != 2 or q_arr.shape != y.shape:
            raise ValueError(f"q and residual must both have shape (N, J), got {q_arr.shape} and {y.shape}")
        joint_count = int(q_arr.shape[1])
        x = build_feature_matrix(q_arr, joint_count)
        n_feat = int(x.shape[1])
        reg = float(alpha) * np.eye(n_feat, dtype=np.float64)
        reg[0, 0] = 0.0
        coefficients = np.zeros((joint_count, n_feat), dtype=np.float64)
        residual_rmse = np.zeros(joint_count, dtype=np.float64)
        a = x.T @ x + reg
        for joint in range(joint_count):
            b = x.T @ y[:, joint]
            coefficients[joint] = np.linalg.solve(a, b)
            pred = x @ coefficients[joint]
            residual_rmse[joint] = float(np.sqrt(np.mean((y[:, joint] - pred) ** 2)))
        return cls(coefficients=coefficients, residual_rmse=residual_rmse, joint_count=joint_count, alpha=float(alpha))

    @classmethod
    def zeros(cls, joint_count: int) -> "StaticBiasModel":
        n_feat = int(trig_features(np.zeros(joint_count), joint_count).shape[0])
        return cls(
            coefficients=np.zeros((joint_count, n_feat), dtype=np.float64),
            residual_rmse=np.zeros(joint_count, dtype=np.float64),
            joint_count=int(joint_count),
        )

    def predict(self, q: np.ndarray) -> np.ndarray:
        q_arr = np.asarray(q, dtype=np.float64)
        single = q_arr.ndim == 1
        if single:
            q_arr = q_arr[None, :]
        x = build_feature_matrix(q_arr, self.joint_count)
        out = x @ self.coefficients.T
        return out[0] if single else out

    def to_dict(self) -> dict[str, Any]:
        return {
            "coefficients": self.coefficients.astype(np.float32),
            "residual_rmse": self.residual_rmse.astype(np.float32),
            "joint_count": self.joint_count,
            "alpha": self.alpha,
        }

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "StaticBiasModel":
        return cls(
            coefficients=np.asarray(data["coefficients"], dtype=np.float64),
            residual_rmse=np.asarray(data.get("residual_rmse", []), dtype=np.float64),
            joint_count=int(data["joint_count"]),
            alpha=float(data.get("alpha", 1.0)),
        )
