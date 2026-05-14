"""Theoretical robot dynamics model."""

from __future__ import annotations

from pathlib import Path
from typing import Any
import os

import numpy as np

from .resolver import ResolvedRobot, write_payload_augmented_urdf


def _import_pybullet():
    try:
        import pybullet as pb  # type: ignore
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "pybullet is required for dynamics.model.TheoreticalModel. "
            "Install pybullet in this environment before running model, torque, or monitor modes."
        ) from exc
    return pb


class TheoreticalModel:
    """Rigid-body dynamics model built from a :class:`ResolvedRobot`.

    Public inputs are SI units:
    - q: rad
    - qd: rad/s
    - qdd: rad/s^2
    - output torque: Nm
    """

    def __init__(self, robot: ResolvedRobot, *, gravity: tuple[float, float, float] = (0.0, 0.0, -9.81), include_joint_dynamics: bool = True):
        self.robot = robot
        self.joint_count = robot.joint_count
        self.include_joint_dynamics = include_joint_dynamics
        self._pb = _import_pybullet()
        self._cid = self._pb.connect(self._pb.DIRECT)
        self._tmp_urdf: Path | None = None
        urdf_path = robot.urdf_path
        if robot.payload is not None and robot.payload.apply_to_model:
            self._tmp_urdf = write_payload_augmented_urdf(robot)
            urdf_path = self._tmp_urdf
        self._pb.setGravity(*gravity, physicsClientId=self._cid)
        self._body = self._pb.loadURDF(str(urdf_path), useFixedBase=True, physicsClientId=self._cid)
        self._joint_indices = self._resolve_pybullet_joint_indices()
        self._damping = np.array([joint.dynamics.damping for joint in robot.active_joints], dtype=np.float64)
        self._friction = np.array([joint.dynamics.friction for joint in robot.active_joints], dtype=np.float64)

    def _resolve_pybullet_joint_indices(self) -> list[int]:
        expected = set(self.robot.joint_names)
        indices: list[int] = []
        for idx in range(self._pb.getNumJoints(self._body, physicsClientId=self._cid)):
            info = self._pb.getJointInfo(self._body, idx, physicsClientId=self._cid)
            name = info[1].decode("utf-8")
            joint_type = info[2]
            if name in expected:
                indices.append(idx)
            elif joint_type in (self._pb.JOINT_REVOLUTE, self._pb.JOINT_PRISMATIC):
                indices.append(idx)
        if len(indices) != self.joint_count:
            raise RuntimeError(
                f"resolved {len(indices)} PyBullet active joints, expected {self.joint_count}; "
                f"URDF={self.robot.urdf_path}"
            )
        return indices

    def _validate_vec(self, values: Any, name: str, default: float | None = None) -> np.ndarray:
        if values is None:
            if default is None:
                raise ValueError(f"{name} is required")
            return np.full(self.joint_count, float(default), dtype=np.float64)
        arr = np.asarray(values, dtype=np.float64)
        if arr.shape != (self.joint_count,):
            raise ValueError(f"{name} must have shape ({self.joint_count},), got {arr.shape}")
        return arr

    def _set_q(self, q_rad: np.ndarray) -> None:
        for i, joint_index in enumerate(self._joint_indices):
            self._pb.resetJointState(self._body, joint_index, float(q_rad[i]), physicsClientId=self._cid)

    def _id(self, q: np.ndarray, qd: np.ndarray, qdd: np.ndarray) -> np.ndarray:
        self._set_q(q)
        tau = self._pb.calculateInverseDynamics(
            self._body,
            list(q),
            list(qd),
            list(qdd),
            physicsClientId=self._cid,
        )
        return np.asarray(tau[: self.joint_count], dtype=np.float64)

    def mass_matrix(self, q: Any) -> np.ndarray:
        q_arr = self._validate_vec(q, "q")
        self._set_q(q_arr)
        matrix = self._pb.calculateMassMatrix(self._body, list(q_arr), physicsClientId=self._cid)
        return np.asarray(matrix, dtype=np.float64)[: self.joint_count, : self.joint_count]

    def gravity(self, q: Any) -> np.ndarray:
        q_arr = self._validate_vec(q, "q")
        zeros = np.zeros(self.joint_count, dtype=np.float64)
        return self._id(q_arr, zeros, zeros)

    def coriolis(self, q: Any, qd: Any) -> np.ndarray:
        q_arr = self._validate_vec(q, "q")
        qd_arr = self._validate_vec(qd, "qd")
        zeros = np.zeros(self.joint_count, dtype=np.float64)
        return self._id(q_arr, qd_arr, zeros) - self._id(q_arr, zeros, zeros)

    def inverse_dynamics(self, q: Any, qd: Any, qdd: Any) -> np.ndarray:
        return self._id(
            self._validate_vec(q, "q"),
            self._validate_vec(qd, "qd"),
            self._validate_vec(qdd, "qdd"),
        )

    def joint_resistance(self, qd: Any, *, velocity_deadband: float = 1e-5) -> np.ndarray:
        qd_arr = self._validate_vec(qd, "qd")
        sign = np.where(np.abs(qd_arr) > velocity_deadband, np.sign(qd_arr), 0.0)
        return self._damping * qd_arr + self._friction * sign

    def estimate_joint_torque(self, q: Any, qd: Any, qdd: Any | None = None) -> np.ndarray:
        q_arr = self._validate_vec(q, "q")
        qd_arr = self._validate_vec(qd, "qd")
        qdd_arr = self._validate_vec(qdd, "qdd", default=0.0)
        tau = self.inverse_dynamics(q_arr, qd_arr, qdd_arr)
        if self.include_joint_dynamics:
            tau = tau + self.joint_resistance(qd_arr)
        return tau

    def close(self) -> None:
        if self._pb.isConnected(self._cid):
            self._pb.disconnect(self._cid)
        if self._tmp_urdf is not None:
            try:
                os.unlink(self._tmp_urdf)
            except OSError:
                pass
            self._tmp_urdf = None

    def __enter__(self) -> "TheoreticalModel":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()


def build_theoretical_model(robot: ResolvedRobot, **kwargs: Any) -> TheoreticalModel:
    return TheoreticalModel(robot, **kwargs)
