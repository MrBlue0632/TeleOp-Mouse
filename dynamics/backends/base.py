"""Common robot backend interface."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Protocol

import numpy as np


@dataclass
class RobotSample:
    timestamp: float
    q: np.ndarray
    qd: np.ndarray
    tau_api: np.ndarray | None = None
    raw: dict | None = None


class RobotBackend(Protocol):
    robot_name: str
    joint_count: int

    def connect(self) -> None:
        ...

    def close(self) -> None:
        ...

    def reset_home(self, home_joints_deg: Iterable[float], gripper_open: float | None = None) -> None:
        ...

    def enter_teach_mode(self, *, sensitivity: int | None = None) -> None:
        ...

    def exit_teach_mode(self) -> None:
        ...

    def read_sample(self) -> RobotSample:
        ...

    def replay_joint_positions(self, q_traj_rad: np.ndarray, timestamps: np.ndarray | None = None, *, speed_deg_s: float = 20.0, acc_deg_s2: float = 200.0) -> None:
        ...

    def stop_motion(self) -> None:
        ...
