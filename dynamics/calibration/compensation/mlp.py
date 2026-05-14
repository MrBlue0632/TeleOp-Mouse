"""Small MLP torque compensation model."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn


class CompensationMLP(nn.Module):
    def __init__(self, input_dim: int, output_dim: int, hidden_dim: int = 64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.SiLU(),
            nn.Linear(hidden_dim, output_dim),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.net(x)


@dataclass
class CompensationBundle:
    model: CompensationMLP
    x_mean: np.ndarray
    x_std: np.ndarray
    y_mean: np.ndarray
    y_std: np.ndarray
    target: str
    joint_count: int

    def _features(self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray) -> np.ndarray:
        return np.concatenate([q, qd, qdd], axis=-1).astype(np.float32)

    def predict_raw(self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray) -> np.ndarray:
        x = self._features(np.asarray(q), np.asarray(qd), np.asarray(qdd))
        if x.ndim == 1:
            x = x[None, :]
        x_norm = (x - self.x_mean) / self.x_std
        self.model.eval()
        with torch.no_grad():
            y_norm = self.model(torch.from_numpy(x_norm.astype(np.float32))).cpu().numpy()
        y = y_norm * self.y_std + self.y_mean
        return y[0] if y.shape[0] == 1 else y

    def predict_compensation(self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray, tau_theory: np.ndarray | None = None) -> np.ndarray:
        pred = self.predict_raw(q, qd, qdd)
        if self.target == "direct_api":
            if tau_theory is None:
                raise ValueError("tau_theory is required when checkpoint target is direct_api")
            return pred - tau_theory
        return pred


def make_features(q: np.ndarray, qd: np.ndarray, qdd: np.ndarray) -> np.ndarray:
    return np.concatenate([q, qd, qdd], axis=1).astype(np.float32)


def train_compensation(
    q: np.ndarray,
    qd: np.ndarray,
    qdd: np.ndarray,
    tau_api: np.ndarray,
    tau_theory: np.ndarray,
    *,
    target: str = "residual",
    epochs: int = 200,
    lr: float = 1e-3,
    hidden_dim: int = 64,
    seed: int = 7,
) -> tuple[CompensationBundle, list[float]]:
    if target not in {"residual", "direct_api"}:
        raise ValueError("target must be residual or direct_api")
    torch.manual_seed(seed)
    joint_count = int(q.shape[1])
    x = make_features(q, qd, qdd)
    y = (tau_api - tau_theory) if target == "residual" else tau_api
    y = y.astype(np.float32)

    x_mean = x.mean(axis=0)
    x_std = x.std(axis=0) + 1e-6
    y_mean = y.mean(axis=0)
    y_std = y.std(axis=0) + 1e-6
    x_norm = (x - x_mean) / x_std
    y_norm = (y - y_mean) / y_std

    model = CompensationMLP(x.shape[1], joint_count, hidden_dim=hidden_dim)
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

    bundle = CompensationBundle(
        model=model,
        x_mean=x_mean.astype(np.float32),
        x_std=x_std.astype(np.float32),
        y_mean=y_mean.astype(np.float32),
        y_std=y_std.astype(np.float32),
        target=target,
        joint_count=joint_count,
    )
    return bundle, losses


def save_compensation(bundle: CompensationBundle, path: str | Path, extra: dict[str, Any] | None = None) -> Path:
    out = Path(path).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    checkpoint = {
        "state_dict": bundle.model.state_dict(),
        "x_mean": bundle.x_mean,
        "x_std": bundle.x_std,
        "y_mean": bundle.y_mean,
        "y_std": bundle.y_std,
        "target": bundle.target,
        "joint_count": bundle.joint_count,
        "input_dim": bundle.joint_count * 3,
        "hidden_dim": bundle.model.net[0].out_features,
        "extra": extra or {},
    }
    torch.save(checkpoint, out)
    return out


def load_compensation(path: str | Path) -> CompensationBundle:
    ckpt = torch.load(Path(path).expanduser(), map_location="cpu", weights_only=False)
    joint_count = int(ckpt["joint_count"])
    model = CompensationMLP(int(ckpt["input_dim"]), joint_count, hidden_dim=int(ckpt.get("hidden_dim", 64)))
    model.load_state_dict(ckpt["state_dict"])
    return CompensationBundle(
        model=model,
        x_mean=np.asarray(ckpt["x_mean"], dtype=np.float32),
        x_std=np.asarray(ckpt["x_std"], dtype=np.float32),
        y_mean=np.asarray(ckpt["y_mean"], dtype=np.float32),
        y_std=np.asarray(ckpt["y_std"], dtype=np.float32),
        target=str(ckpt.get("target", "residual")),
        joint_count=joint_count,
    )
