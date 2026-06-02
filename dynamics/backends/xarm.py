"""xArm SDK backend."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable
import time

import numpy as np

from .base import RobotSample


def _ret_ok(ret: Any) -> bool:
    return isinstance(ret, (list, tuple)) and len(ret) >= 2 and int(ret[0]) == 0


def _as_array(values: Any, count: int, *, default: float = 0.0) -> np.ndarray:
    if values is None:
        return np.full(count, default, dtype=np.float64)
    arr = np.asarray(list(values)[:count], dtype=np.float64)
    if arr.shape[0] < count:
        arr = np.pad(arr, (0, count - arr.shape[0]), constant_values=default)
    return arr


def _coerce_speed_to_rad(values: Any, count: int) -> np.ndarray:
    arr = _as_array(values, count)
    # xArm cached realtime speeds may follow the SDK default unit.  Values above
    # plausible rad/s are treated as deg/s.
    if arr.size and np.nanmax(np.abs(arr)) > 2.0 * np.pi:
        arr = np.deg2rad(arr)
    return arr


def _payload_tcp_load(config: dict[str, Any]) -> tuple[float, tuple[float, float, float]] | None:
    """Return xArm TCP load settings from the configured payload profile."""
    from dynamics.resolver import payload_from_config

    payload = payload_from_config(config.get("payload"))
    if payload is None:
        return None
    center_of_gravity_mm = tuple(float(value) * 1000.0 for value in payload.com_xyz_m)
    return float(payload.mass_kg), center_of_gravity_mm


@dataclass
class XArmBackend:
    ip: str
    joint_count: int = 6
    robot_name: str = "xarm6"
    robot_port: int = 502
    report_port_normal: int = 30001
    report_port_rich: int = 30002
    report_port_real: int = 30003
    sdk_timeout_s: float = 0.5
    tcp_load: tuple[float, tuple[float, float, float]] | None = None

    arm: Any | None = None

    @classmethod
    def from_config(cls, config: dict[str, Any]) -> "XArmBackend":
        conn = config.get("connection", {}) or {}
        return cls(
            ip=str(conn.get("ip", "192.168.1.199")),
            joint_count=int(config.get("joint_count", 6)),
            robot_name=str(config.get("robot_name", "xarm6")),
            robot_port=int(conn.get("robot_port", 502)),
            report_port_normal=int(conn.get("report_port_normal", 30001)),
            report_port_rich=int(conn.get("report_port_rich", 30002)),
            report_port_real=int(conn.get("report_port_real", 30003)),
            sdk_timeout_s=float(conn.get("sdk_timeout_s", 0.5)),
            tcp_load=_payload_tcp_load(config),
        )

    def connect(self) -> None:
        from xarm.core.config.x_config import XCONF
        from xarm.wrapper import XArmAPI

        XCONF.SocketConf.TCP_CONTROL_PORT = int(self.robot_port)
        XCONF.SocketConf.TCP_REPORT_NORM_PORT = int(self.report_port_normal)
        XCONF.SocketConf.TCP_REPORT_RICH_PORT = int(self.report_port_rich)
        XCONF.SocketConf.TCP_REPORT_REAL_PORT = int(self.report_port_real)

        self.arm = XArmAPI(self.ip, do_not_open=True)
        self.arm.connect()
        if not self.arm.connected:
            raise RuntimeError(f"failed to connect xArm at {self.ip}:{self.robot_port}")
        self.arm.set_timeout(self.sdk_timeout_s)
        self._ensure_ready()
        self._apply_tcp_load()
        try:
            self.arm.set_report_tau_or_i(tau_or_i=0)
        except Exception:
            pass

    def _require_arm(self) -> Any:
        if self.arm is None:
            raise RuntimeError("xArm backend is not connected")
        return self.arm

    def _ensure_ready(self) -> None:
        arm = self._require_arm()
        try:
            arm.clean_warn()
            arm.clean_error()
            arm.motion_enable(True)
            arm.set_state(0)
        except Exception:
            pass

    def _apply_tcp_load(self) -> None:
        if self.tcp_load is None:
            return
        arm = self._require_arm()
        weight_kg, center_of_gravity_mm = self.tcp_load
        ret = arm.set_tcp_load(float(weight_kg), list(center_of_gravity_mm), wait=True)
        if isinstance(ret, int) and ret != 0:
            raise RuntimeError(f"xArm set_tcp_load failed with code {ret}")

    def close(self) -> None:
        if self.arm is not None:
            try:
                self.stop_motion()
            except Exception:
                pass
            self.arm.disconnect()
            self.arm = None

    def reset_home(self, home_joints_deg: Iterable[float], gripper_open: float | None = None) -> None:
        arm = self._require_arm()
        self._ensure_ready()
        arm.set_mode(0)
        arm.set_state(0)
        ret = arm.set_servo_angle(
            angle=list(home_joints_deg),
            speed=20,
            mvacc=200,
            is_radian=False,
            wait=True,
        )
        if isinstance(ret, int) and ret != 0:
            raise RuntimeError(f"xArm set_servo_angle failed with code {ret}")
        if gripper_open is not None:
            try:
                arm.set_gripper_enable(True)
                arm.set_gripper_position(float(gripper_open), wait=False, speed=2400, auto_enable=True)
            except Exception:
                pass

    def enter_teach_mode(self, *, sensitivity: int | None = None) -> None:
        arm = self._require_arm()
        self._ensure_ready()
        if sensitivity is not None:
            arm.set_teach_sensitivity(int(sensitivity))
        arm.set_mode(2)
        arm.set_state(0)

    def exit_teach_mode(self) -> None:
        arm = self._require_arm()
        arm.set_mode(0)
        arm.set_state(0)

    def read_sample(self) -> RobotSample:
        arm = self._require_arm()
        timestamp = time.time()
        q = None
        qd = None
        tau = None

        try:
            ret = arm.get_joint_states(is_radian=True)
            if _ret_ok(ret):
                states = ret[1]
                if len(states) >= 1:
                    q = _as_array(states[0], self.joint_count)
                if len(states) >= 2:
                    qd = _as_array(states[1], self.joint_count)
                if len(states) >= 3:
                    tau = _as_array(states[2], self.joint_count)
        except Exception:
            pass

        if q is None:
            ret = arm.get_servo_angle(is_radian=True)
            if _ret_ok(ret):
                q = _as_array(ret[1], self.joint_count)
        if qd is None:
            qd = _coerce_speed_to_rad(getattr(arm, "realtime_joint_speeds", None), self.joint_count)
        if tau is None:
            try:
                ret = arm.get_joints_torque()
                if _ret_ok(ret):
                    tau = _as_array(ret[1], self.joint_count)
            except Exception:
                pass
        if tau is None:
            tau = _as_array(getattr(arm, "joints_torque", None), self.joint_count)
        if q is None:
            raise RuntimeError("failed to read xArm joint positions")

        return RobotSample(timestamp=timestamp, q=q, qd=qd, tau_api=tau, raw={})

    def replay_joint_positions(self, q_traj_rad: np.ndarray, timestamps: np.ndarray | None = None, *, speed_deg_s: float = 20.0, acc_deg_s2: float = 200.0) -> None:
        arm = self._require_arm()
        q_traj = np.asarray(q_traj_rad, dtype=np.float64)
        if q_traj.ndim != 2 or q_traj.shape[1] != self.joint_count:
            raise ValueError(f"q_traj must have shape (N, {self.joint_count})")
        ts = None
        if timestamps is not None:
            ts = np.asarray(timestamps, dtype=np.float64)
            if ts.ndim != 1 or len(ts) != len(q_traj):
                raise ValueError("timestamps must be a 1-D array with the same length as q_traj")
            if not np.all(np.isfinite(ts)):
                raise ValueError("timestamps must contain only finite values")

        if ts is not None and len(q_traj) > 1:
            arm.set_mode(1)
            arm.set_state(0)
            time.sleep(0.1)
            t0 = time.monotonic()
            ts0 = float(ts[0])
            for idx, q in enumerate(q_traj):
                if idx > 0:
                    target = float(ts[idx]) - ts0
                    delay = target - (time.monotonic() - t0)
                    if delay > 0:
                        time.sleep(delay)
                ret = arm.set_servo_angle_j(
                    angles=q.tolist(),
                    speed=float(speed_deg_s),
                    mvacc=float(acc_deg_s2),
                    mvtime=0,
                    is_radian=True,
                )
                if isinstance(ret, int) and ret != 0:
                    raise RuntimeError(f"xArm set_servo_angle_j failed with code {ret}")
            return

        arm.set_mode(0)
        arm.set_state(0)
        for q in q_traj:
            ret = arm.set_servo_angle(
                angle=np.rad2deg(q).tolist(),
                speed=float(speed_deg_s),
                mvacc=float(acc_deg_s2),
                is_radian=False,
                wait=False,
            )
            if isinstance(ret, int) and ret != 0:
                raise RuntimeError(f"xArm set_servo_angle failed with code {ret}")

    def stop_motion(self) -> None:
        arm = self._require_arm()
        try:
            arm.vc_set_joint_velocity([0.0] * self.joint_count, is_radian=True, duration=0)
        except Exception:
            try:
                arm.set_state(4)
            except Exception:
                pass
