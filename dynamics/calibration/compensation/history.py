"""Kinematic-history torque compensation model."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import torch
from torch import nn


CHANNELS_Q_QD = "q_qd"
CHANNELS_Q_QD_QDD = "q_qd_qdd"
KINEMATIC_HISTORY_KIND = "kinematic_history"
DEFAULT_WINDOW_POINTS = 20
DEFAULT_CONTROL_HZ = 30.0


class KinematicHistoryCNN(nn.Module):
    def __init__(self, input_channels: int, output_dim: int, hidden_dim: int = 64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Conv1d(input_channels, hidden_dim, kernel_size=3, padding=1),
            nn.SiLU(),
            nn.Conv1d(hidden_dim, hidden_dim, kernel_size=3, padding=1),
            nn.SiLU(),
            nn.Conv1d(hidden_dim, hidden_dim, kernel_size=3, padding=1),
            nn.SiLU(),
        )
        self.head = nn.Linear(hidden_dim, output_dim)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        y = self.net(x).mean(dim=-1)
        return self.head(y)


def _as_matrix(values: np.ndarray, joint_count: int | None, name: str) -> np.ndarray:
    arr = np.asarray(values, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr[None, :]
    if arr.ndim != 2:
        raise ValueError(f"{name} must have shape (N, J), got {arr.shape}")
    if joint_count is not None and arr.shape[1] != joint_count:
        raise ValueError(f"{name} must have shape (N, {joint_count}), got {arr.shape}")
    return arr


def _validate_channels(channels: str) -> str:
    out = str(channels)
    if out not in {CHANNELS_Q_QD, CHANNELS_Q_QD_QDD}:
        raise ValueError("channels must be q_qd or q_qd_qdd")
    return out


def _history_stride(control_hz: float) -> int:
    hz = float(control_hz)
    if hz <= 0:
        raise ValueError("control_hz must be > 0")
    stride = hz / 10.0
    rounded = int(round(stride))
    if rounded <= 0 or not np.isclose(stride, rounded, rtol=0.0, atol=1e-9):
        raise ValueError("control_hz must be an integer multiple of 10 Hz")
    return rounded


def _channel_count(joint_count: int, channels: str) -> int:
    return int(joint_count) * (3 if _validate_channels(channels) == CHANNELS_Q_QD_QDD else 2)


def _stack_channels(q: np.ndarray, qd: np.ndarray, qdd: np.ndarray, channels: str) -> np.ndarray:
    selected = [q, qd]
    if _validate_channels(channels) == CHANNELS_Q_QD_QDD:
        selected.append(qdd)
    return np.concatenate(selected, axis=0)


def build_history_windows(
    q: np.ndarray,
    qd: np.ndarray,
    qdd: np.ndarray | None = None,
    *,
    control_hz: float = DEFAULT_CONTROL_HZ,
    channels: str = CHANNELS_Q_QD,
    window_points: int = DEFAULT_WINDOW_POINTS,
) -> tuple[np.ndarray, np.ndarray]:
    """Build windows shaped (N, C, T), with time ordered current to oldest."""
    q_arr = _as_matrix(q, None, "q")
    joint_count = int(q_arr.shape[1])
    qd_arr = _as_matrix(qd, joint_count, "qd")
    qdd_arr = np.zeros_like(q_arr) if qdd is None else _as_matrix(qdd, joint_count, "qdd")
    if not (q_arr.shape == qd_arr.shape == qdd_arr.shape):
        raise ValueError("q, qd, and qdd must have matching shape")
    ch = _validate_channels(channels)
    points = int(window_points)
    if points <= 0:
        raise ValueError("window_points must be > 0")
    stride = _history_stride(control_hz)
    first_index = points * stride - 1
    if q_arr.shape[0] <= first_index:
        empty = np.zeros((0, _channel_count(joint_count, ch), points), dtype=np.float32)
        return empty, np.zeros(0, dtype=np.int64)

    indices = np.arange(first_index, q_arr.shape[0], dtype=np.int64)
    windows = np.zeros((indices.shape[0], _channel_count(joint_count, ch), points), dtype=np.float32)
    for row, current in enumerate(indices):
        for lag in range(points):
            src = int(current) - lag * stride
            windows[row, :, lag] = _stack_channels(q_arr[src], qd_arr[src], qdd_arr[src], ch)
    return windows, indices


@dataclass
class KinematicHistoryEstimate:
    tau_external: np.ndarray
    tau_static_bias: np.ndarray
    tau_motion_comp: np.ndarray
    tau_firmware_bias: np.ndarray
    tau_comp: np.ndarray
    is_ready: bool
    time_since_stop: np.ndarray | None = None
    firmware_state: np.ndarray | None = None
    motion_lambda: float | None = None
    is_moving: np.ndarray | None = None


@dataclass
class KinematicHistoryCompensator:
    model: KinematicHistoryCNN
    x_mean: np.ndarray
    x_std: np.ndarray
    y_mean: np.ndarray
    y_std: np.ndarray
    joint_count: int
    channels: str = CHANNELS_Q_QD
    control_hz: float = DEFAULT_CONTROL_HZ
    window_points: int = DEFAULT_WINDOW_POINTS
    hidden_dim: int = 64
    metadata: dict[str, Any] | None = None

    kind = KINEMATIC_HISTORY_KIND

    def __post_init__(self) -> None:
        self.channels = _validate_channels(self.channels)
        self._stride = _history_stride(self.control_hz)
        self._q_history: list[np.ndarray] = []
        self._qd_history: list[np.ndarray] = []
        self._qdd_history: list[np.ndarray] = []

    @classmethod
    def zeros(
        cls,
        joint_count: int,
        *,
        channels: str = CHANNELS_Q_QD,
        control_hz: float = DEFAULT_CONTROL_HZ,
        window_points: int = DEFAULT_WINDOW_POINTS,
        hidden_dim: int = 64,
    ) -> "KinematicHistoryCompensator":
        ch = _validate_channels(channels)
        input_channels = _channel_count(int(joint_count), ch)
        model = KinematicHistoryCNN(input_channels, int(joint_count), hidden_dim=hidden_dim)
        for param in model.parameters():
            nn.init.zeros_(param)
        return cls(
            model=model,
            x_mean=np.zeros((input_channels, 1), dtype=np.float32),
            x_std=np.ones((input_channels, 1), dtype=np.float32),
            y_mean=np.zeros(int(joint_count), dtype=np.float32),
            y_std=np.ones(int(joint_count), dtype=np.float32),
            joint_count=int(joint_count),
            channels=ch,
            control_hz=float(control_hz),
            window_points=int(window_points),
            hidden_dim=int(hidden_dim),
        )

    @property
    def is_ready(self) -> bool:
        return len(self._q_history) >= self.window_points * self._stride

    def reset(self) -> None:
        self._q_history.clear()
        self._qd_history.clear()
        self._qdd_history.clear()

    def _remember(self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray) -> None:
        self._q_history.append(np.asarray(q, dtype=np.float64)[: self.joint_count].copy())
        self._qd_history.append(np.asarray(qd, dtype=np.float64)[: self.joint_count].copy())
        self._qdd_history.append(np.asarray(qdd, dtype=np.float64)[: self.joint_count].copy())
        max_len = self.window_points * self._stride
        del self._q_history[:-max_len]
        del self._qd_history[:-max_len]
        del self._qdd_history[:-max_len]

    def _current_window(self) -> np.ndarray | None:
        if not self.is_ready:
            return None
        window = np.zeros((_channel_count(self.joint_count, self.channels), self.window_points), dtype=np.float32)
        last = len(self._q_history) - 1
        for lag in range(self.window_points):
            src = last - lag * self._stride
            window[:, lag] = _stack_channels(
                self._q_history[src],
                self._qd_history[src],
                self._qdd_history[src],
                self.channels,
            )
        return window

    def predict_window(self, window: np.ndarray) -> np.ndarray:
        x = np.asarray(window, dtype=np.float32)
        if x.ndim == 2:
            x = x[None, :, :]
        x_norm = (x - self.x_mean[None, :, :]) / self.x_std[None, :, :]
        self.model.eval()
        with torch.inference_mode():
            y_norm = self.model(torch.from_numpy(x_norm.astype(np.float32))).cpu().numpy()
        y = y_norm * self.y_std[None, :] + self.y_mean[None, :]
        return y[0] if y.shape[0] == 1 else y

    def predict_compensation(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        qdd: np.ndarray,
        tau_theory: np.ndarray | None = None,
    ) -> np.ndarray:
        self._remember(q, qd, qdd)
        window = self._current_window()
        if window is None:
            return np.zeros(self.joint_count, dtype=np.float64)
        return np.asarray(self.predict_window(window), dtype=np.float64)

    def update(
        self,
        q: np.ndarray,
        qd: np.ndarray,
        qdd: np.ndarray,
        tau_api: np.ndarray,
        tau_model: np.ndarray,
        timestamp: float | None = None,
    ) -> KinematicHistoryEstimate:
        tau_api_arr = np.asarray(tau_api, dtype=np.float64)[: self.joint_count]
        tau_model_arr = np.asarray(tau_model, dtype=np.float64)[: self.joint_count]
        tau_comp = self.predict_compensation(q, qd, qdd)
        ready = self.is_ready
        zeros = np.zeros(self.joint_count, dtype=np.float64)
        tau_external = tau_api_arr - tau_model_arr - tau_comp
        return KinematicHistoryEstimate(
            tau_external=tau_external,
            tau_static_bias=zeros.copy(),
            tau_motion_comp=tau_comp.copy(),
            tau_firmware_bias=zeros.copy(),
            tau_comp=tau_comp,
            is_ready=ready,
            time_since_stop=zeros.copy(),
            firmware_state=zeros.copy(),
            motion_lambda=None,
            is_moving=np.abs(np.asarray(qd, dtype=np.float64)[: self.joint_count]) > 0.0,
        )

    def checkpoint(self) -> dict[str, Any]:
        return {
            "kind": self.kind,
            "state_dict": self.model.state_dict(),
            "x_mean": self.x_mean.astype(np.float32),
            "x_std": self.x_std.astype(np.float32),
            "y_mean": self.y_mean.astype(np.float32),
            "y_std": self.y_std.astype(np.float32),
            "joint_count": self.joint_count,
            "channels": self.channels,
            "control_hz": self.control_hz,
            "window_points": self.window_points,
            "hidden_dim": self.hidden_dim,
            "input_channels": _channel_count(self.joint_count, self.channels),
            "metadata": self.metadata or {},
        }

    def save(self, path: str | Path) -> Path:
        out = Path(path).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        torch.save(self.checkpoint(), out)
        return out

    @classmethod
    def from_checkpoint(cls, checkpoint: dict[str, Any]) -> "KinematicHistoryCompensator":
        joint_count = int(checkpoint["joint_count"])
        channels = str(checkpoint.get("channels", CHANNELS_Q_QD))
        hidden_dim = int(checkpoint.get("hidden_dim", 64))
        input_channels = int(checkpoint.get("input_channels", _channel_count(joint_count, channels)))
        model = KinematicHistoryCNN(input_channels, joint_count, hidden_dim=hidden_dim)
        model.load_state_dict(checkpoint["state_dict"])
        return cls(
            model=model,
            x_mean=np.asarray(checkpoint["x_mean"], dtype=np.float32),
            x_std=np.asarray(checkpoint["x_std"], dtype=np.float32),
            y_mean=np.asarray(checkpoint["y_mean"], dtype=np.float32),
            y_std=np.asarray(checkpoint["y_std"], dtype=np.float32),
            joint_count=joint_count,
            channels=channels,
            control_hz=float(checkpoint.get("control_hz", DEFAULT_CONTROL_HZ)),
            window_points=int(checkpoint.get("window_points", DEFAULT_WINDOW_POINTS)),
            hidden_dim=hidden_dim,
            metadata=dict(checkpoint.get("metadata", {})),
        )

    @classmethod
    def load(cls, path: str | Path) -> "KinematicHistoryCompensator":
        checkpoint = torch.load(Path(path).expanduser(), map_location="cpu", weights_only=False)
        if not isinstance(checkpoint, dict) or checkpoint.get("kind") != cls.kind:
            raise ValueError(f"not a kinematic-history compensation checkpoint: {path}")
        return cls.from_checkpoint(checkpoint)


def train_kinematic_history_model(
    *,
    q: np.ndarray,
    qd: np.ndarray,
    qdd: np.ndarray,
    tau_api: np.ndarray,
    tau_model: np.ndarray,
    channels: str = CHANNELS_Q_QD,
    control_hz: float = DEFAULT_CONTROL_HZ,
    window_points: int = DEFAULT_WINDOW_POINTS,
    epochs: int = 200,
    lr: float = 1e-3,
    hidden_dim: int = 64,
    seed: int = 7,
    embodiment: dict[str, Any] | None = None,
) -> tuple[KinematicHistoryCompensator, dict[str, Any]]:
    q_arr = _as_matrix(q, None, "q")
    joint_count = int(q_arr.shape[1])
    qd_arr = _as_matrix(qd, joint_count, "qd")
    qdd_arr = _as_matrix(qdd, joint_count, "qdd")
    tau_api_arr = _as_matrix(tau_api, joint_count, "tau_api")
    tau_model_arr = _as_matrix(tau_model, joint_count, "tau_model")
    x, indices = build_history_windows(
        q_arr,
        qd_arr,
        qdd_arr,
        control_hz=control_hz,
        channels=channels,
        window_points=window_points,
    )
    if x.shape[0] == 0:
        raise ValueError("not enough samples for kinematic-history window")
    target = (tau_api_arr - tau_model_arr)[indices].astype(np.float32)

    torch.manual_seed(seed)
    x_mean = x.mean(axis=(0, 2), keepdims=False).astype(np.float32)[:, None]
    x_std = (x.std(axis=(0, 2), keepdims=False) + 1e-6).astype(np.float32)[:, None]
    y_mean = target.mean(axis=0).astype(np.float32)
    y_std = (target.std(axis=0) + 1e-6).astype(np.float32)
    x_norm = (x - x_mean[None, :, :]) / x_std[None, :, :]
    y_norm = (target - y_mean[None, :]) / y_std[None, :]

    model = KinematicHistoryCNN(x.shape[1], joint_count, hidden_dim=hidden_dim)
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

    metadata: dict[str, Any] = {
        "joint_count": joint_count,
        "target": "tau_api_minus_tau_model",
        "channels": _validate_channels(channels),
        "control_hz": float(control_hz),
        "window_points": int(window_points),
        "history_seconds": float(window_points) * 0.1,
        "final_loss": losses[-1] if losses else None,
        "loss": "per_joint_target_normalized_mse",
    }
    if embodiment is not None:
        metadata["embodiment"] = embodiment
    bundle = KinematicHistoryCompensator(
        model=model,
        x_mean=x_mean,
        x_std=x_std,
        y_mean=y_mean,
        y_std=y_std,
        joint_count=joint_count,
        channels=_validate_channels(channels),
        control_hz=float(control_hz),
        window_points=int(window_points),
        hidden_dim=int(hidden_dim),
        metadata=metadata,
    )
    return bundle, metadata
