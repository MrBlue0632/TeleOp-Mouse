#!/usr/bin/env python3
"""Run safe-workspace xArm motion with live force and 3D visualization."""

from __future__ import annotations

import argparse
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
from typing import Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from dynamics.calibration.workspace import generate_safe_joint_trajectory
from dynamics.config import load_config

DEFAULT_AMPLITUDE_DEG = (8.0, 6.0, 6.0, 10.0, 8.0, 8.0)
DEFAULT_TORQUE_MODEL = REPO_ROOT / "dynamics" / "calibration" / "compensation" / "history_q_qd.pt"


@dataclass(frozen=True)
class SafeMotionPlan:
    q_traj_rad: np.ndarray
    timestamps: np.ndarray
    home_joints_deg: list[float]
    amplitude_deg: list[float]
    speed_deg_s: float
    accel_deg_s2: float
    cycles: int
    return_home_on_exit: bool


class _NullDataset:
    """Dataset-shaped sink used when this demo is visualization-only."""

    def __init__(self, data_dir: str | os.PathLike[str]) -> None:
        self.root = str(Path(data_dir).expanduser() / "safe_workspace_visualization_live")
        self.num_episodes = 0
        self.meta = SimpleNamespace(total_frames=0)
        self.episode_buffer = {"episode_index": None, "size": 0}

    def add_frame(self, _frame: dict) -> None:
        return None

    def save_episode(self) -> None:
        return None

    def finalize(self) -> None:
        return None


def _parse_float_csv(text: str, *, expected: int, name: str) -> list[float]:
    try:
        values = [float(part.strip()) for part in str(text).split(",") if part.strip()]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"{name} must contain comma-separated floats") from exc
    if len(values) != expected:
        raise argparse.ArgumentTypeError(f"{name} must contain exactly {expected} values")
    if not all(np.isfinite(values)):
        raise argparse.ArgumentTypeError(f"{name} must contain only finite values")
    return values


def parse_amplitude(text: str) -> list[float]:
    values = _parse_float_csv(text, expected=6, name="--amplitude-deg")
    if any(value <= 0.0 for value in values):
        raise argparse.ArgumentTypeError("--amplitude-deg values must be > 0")
    return values


def _default_torque_model() -> str | None:
    env_value = os.getenv("TELEOP_TORQUE_COMP_MODEL")
    if env_value:
        return env_value
    return str(DEFAULT_TORQUE_MODEL) if DEFAULT_TORQUE_MODEL.is_file() else None


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default="xarm6", help="robot config name")
    parser.add_argument("--config", help="optional robot config YAML path")
    parser.add_argument("--robot-ip", default=os.getenv("ROBOT_IP"), help="xArm IP override")
    parser.add_argument("--robot-port", type=int, help="xArm control port override")
    parser.add_argument("--report-port-normal", type=int, help="xArm normal report port override")
    parser.add_argument("--report-port-rich", type=int, help="xArm rich report port override")
    parser.add_argument("--report-port-real", type=int, help="xArm real-time report port override")
    parser.add_argument("--sdk-timeout-s", type=float, help="xArm SDK timeout override")
    parser.add_argument("--rate-hz", type=float, default=30.0, help="state/data loop frequency")
    parser.add_argument("--control-hz", type=float, default=50.0, help="joint target streaming frequency")
    parser.add_argument("--data-dir", default=str(REPO_ROOT / "data"), help="dashboard/dataset data directory")
    parser.add_argument("--camera-id", type=int, default=10)
    parser.add_argument("--camera-dev", default="/dev/video10")
    parser.add_argument("--base-camera-id", type=int, default=16)
    parser.add_argument("--base-camera-dev", default="/dev/video16")
    parser.add_argument("--target-camera-id", type=int, default=4)
    parser.add_argument("--target-camera-dev", default="/dev/video4")
    parser.add_argument("--display-camera", default="wrist", choices=["wrist", "base", "target"])
    parser.add_argument("--allow-camera-fallback", dest="allow_camera_fallback", action="store_true")
    parser.add_argument("--strict-camera-dev", dest="allow_camera_fallback", action="store_false")
    parser.set_defaults(allow_camera_fallback=True)
    parser.add_argument("--show-video", dest="show_video", action="store_true", help="show local OpenCV camera/torque window")
    parser.add_argument("--no-video", dest="show_video", action="store_false")
    parser.set_defaults(show_video=False)
    parser.add_argument("--web-dashboard", dest="web_dashboard", action="store_true")
    parser.add_argument("--no-web-dashboard", dest="web_dashboard", action="store_false")
    parser.set_defaults(web_dashboard=True)
    parser.add_argument("--web-host", default="127.0.0.1")
    parser.add_argument("--web-port", type=int, default=8765)
    parser.add_argument("--web-fps", type=float, default=10.0)
    parser.add_argument("--repo-id", default="teleop/xarm_safe_workspace_visualization")
    parser.add_argument("--task", default="safe_workspace_force_visualization")
    parser.add_argument("--vcodec", default="libsvtav1")
    parser.add_argument("--encoder-threads", type=int, default=2)
    parser.add_argument("--encoder-queue-maxsize", type=int, default=30)
    parser.add_argument("--torque-comp-model", default=_default_torque_model(), help="optional torque compensation checkpoint")
    parser.add_argument("--no-torque-comp", action="store_true", help="force firmware-bias fallback torque observation")
    parser.add_argument("--record-dataset", action="store_true", help="store LeRobot frames instead of visualization-only telemetry")
    parser.add_argument("--no-start-reset", action="store_true", help="skip startup home reset")
    parser.add_argument("--motion-duration-s", type=float, default=45.0, help="minimum duration of one generated motion cycle")
    parser.add_argument("--motion-hz", type=float, default=50.0, help="safe trajectory generation frequency")
    parser.add_argument("--amplitude-deg", type=parse_amplitude, default=list(DEFAULT_AMPLITUDE_DEG), help="six comma-separated joint amplitudes around home")
    parser.add_argument("--speed-deg-s", type=float, default=8.0, help="max joint speed for generated safe trajectory")
    parser.add_argument("--accel-deg-s2", type=float, default=35.0, help="max joint acceleration for generated safe trajectory")
    parser.add_argument("--hold-s", type=float, default=1.0, help="hold time at generated waypoints")
    parser.add_argument("--waypoints", type=int, default=8, help="generated safe waypoints per cycle")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--cycles", type=int, default=0, help="0 means repeat until Ctrl+C")
    parser.add_argument("--no-return-home", action="store_true", help="do not return home after stopping")
    parser.add_argument("--self-check", action="store_true", help="validate launch config without connecting to the robot")
    return parser.parse_args(argv)


def _config_overrides(args: argparse.Namespace) -> dict:
    connection: dict[str, object] = {}
    if args.robot_ip:
        connection["ip"] = args.robot_ip
    if args.robot_port is not None:
        connection["robot_port"] = args.robot_port
    if args.report_port_normal is not None:
        connection["report_port_normal"] = args.report_port_normal
    if args.report_port_rich is not None:
        connection["report_port_rich"] = args.report_port_rich
    if args.report_port_real is not None:
        connection["report_port_real"] = args.report_port_real
    if args.sdk_timeout_s is not None:
        connection["sdk_timeout_s"] = args.sdk_timeout_s
    return {"connection": connection} if connection else {}


def build_motion_plan(args: argparse.Namespace, config: dict) -> SafeMotionPlan:
    joint_count = int(config.get("joint_count", 6))
    if joint_count != 6:
        raise ValueError(f"safe visualization runner expects a 6-DOF arm, got joint_count={joint_count}")
    home = np.asarray(config.get("home_joints_deg", []), dtype=np.float64)
    if home.size < 6 or not np.all(np.isfinite(home[:6])):
        raise ValueError("config home_joints_deg must contain six finite values")
    if args.motion_hz <= 0.0 or args.speed_deg_s <= 0.0 or args.accel_deg_s2 <= 0.0:
        raise ValueError("motion_hz, speed_deg_s, and accel_deg_s2 must be > 0")
    if args.motion_duration_s <= 0.0:
        raise ValueError("motion_duration_s must be > 0")
    if args.waypoints < 1:
        raise ValueError("waypoints must be >= 1")
    if args.cycles < 0:
        raise ValueError("cycles must be >= 0")

    amplitude = np.asarray(args.amplitude_deg, dtype=np.float64)
    q_traj, timestamps = generate_safe_joint_trajectory(
        np.deg2rad(home[:6]),
        duration_s=float(args.motion_duration_s),
        hz=float(args.motion_hz),
        amplitude_deg=amplitude,
        speed_deg_s=float(args.speed_deg_s),
        accel_deg_s2=float(args.accel_deg_s2),
        hold_s=max(0.0, float(args.hold_s)),
        waypoint_count=int(args.waypoints),
        seed=int(args.seed),
    )
    q_deg = np.rad2deg(q_traj)
    delta = np.abs(q_deg - home[:6])
    if np.nanmax(delta - amplitude) > 1e-6:
        raise ValueError("generated trajectory exceeds configured joint amplitude envelope")
    if not np.allclose(q_deg[-1], home[:6], atol=1e-6):
        raise ValueError("generated trajectory must end at home")
    return SafeMotionPlan(
        q_traj_rad=q_traj,
        timestamps=timestamps,
        home_joints_deg=[float(v) for v in home[:6]],
        amplitude_deg=[float(v) for v in amplitude],
        speed_deg_s=float(args.speed_deg_s),
        accel_deg_s2=float(args.accel_deg_s2),
        cycles=int(args.cycles),
        return_home_on_exit=not bool(args.no_return_home),
    )


def _print_plan(plan: SafeMotionPlan, config: dict, args: argparse.Namespace) -> None:
    connection = config.get("connection", {}) or {}
    url = f"http://{args.web_host}:{args.web_port}/" if args.web_dashboard else "disabled"
    cycles = "infinite" if plan.cycles == 0 else str(plan.cycles)
    print("============================================")
    print("  Safe Workspace Force Visualization")
    print("============================================")
    print(f"  Robot IP      : {connection.get('ip')}:{connection.get('robot_port', 502)}")
    print(f"  Web Dashboard : {url}")
    print("  EEF Force HUD : ee_force_display in EEF frame")
    print(f"  Torque Model  : {args.torque_comp_model or 'firmware-bias fallback'}")
    print(f"  Record Data   : {'yes' if args.record_dataset else 'no (telemetry only)'}")
    print(f"  Home Joints   : {', '.join(f'{v:.1f}' for v in plan.home_joints_deg)} deg")
    print(f"  Amplitude     : {', '.join(f'{v:.1f}' for v in plan.amplitude_deg)} deg")
    print(f"  Speed / Accel : {plan.speed_deg_s:.1f} deg/s / {plan.accel_deg_s2:.1f} deg/s^2")
    print(f"  Samples       : {len(plan.q_traj_rad)} @ {args.motion_hz:.1f} Hz, cycles={cycles}")
    print("============================================")


def _make_safe_app_class(teleop_app_cls):
    class SafeWorkspaceMotionApp(teleop_app_cls):
        def __init__(self, *init_args, safe_motion_plan: SafeMotionPlan, record_dataset: bool, **init_kwargs):
            self.safe_motion_plan = safe_motion_plan
            self.record_dataset = bool(record_dataset)
            self.safe_motion_cycles_done = 0
            self.safe_motion_point_index = 0
            self.safe_motion_mode = "initializing"
            super().__init__(*init_args, **init_kwargs)

        def _init_lerobot_dataset(self):
            if self.record_dataset:
                return super()._init_lerobot_dataset()
            self.lerobot_dataset = _NullDataset(self.data_dir)
            print(f"[INFO] dataset recording disabled; live telemetry sink={self.lerobot_dataset.root}")

        def start_input(self):
            self.camera = self.open_camera_with_probe(self.camera_id, self.camera_dev, allow_fallback=True, role="wrist")
            if self.camera is None:
                print("[WARN] no wrist camera frame available")
                if self.strict_camera_dev:
                    raise RuntimeError(f"strict camera mode: cannot open {self.camera_dev}")
            self.base_camera = self.open_camera_with_probe(self.base_camera_id, self.base_camera_dev, allow_fallback=False, role="base")
            if self.base_camera is None:
                print("[WARN] no base camera frame available")
            self.target_camera = self.open_camera_with_probe(self.target_camera_id, self.target_camera_dev, allow_fallback=False, role="target")
            if self.target_camera is None:
                print("[WARN] no target camera frame available")

        def _enter_servo_joint_mode(self) -> bool:
            try:
                self.stop_motion_now()
            except Exception:
                pass
            for _ in range(3):
                try:
                    self.arm.set_state(0)
                    time.sleep(0.03)
                    self.arm.set_mode(1)
                    time.sleep(0.05)
                    self.arm.set_state(0)
                    time.sleep(0.03)
                    mode = int(getattr(self.arm, "mode", 1))
                    state = int(getattr(self.arm, "state", 0))
                    if mode == 1 and state in (0, 1, 2):
                        return True
                except Exception:
                    time.sleep(0.05)
            return False

        def stop_motion_now(self):
            try:
                mode = int(getattr(self.arm, "mode", -1))
            except Exception:
                mode = -1
            if mode == 1:
                try:
                    self.arm.set_state(0)
                    return
                except Exception:
                    pass
            return super().stop_motion_now()

        def _send_joint_target(self, q_deg: np.ndarray) -> None:
            t0 = time.monotonic()
            ret = self.arm.set_servo_angle_j(q_deg.tolist(), is_radian=False)
            dt_ms = (time.monotonic() - t0) * 1000.0
            self.diag_send_cnt += 1
            self.diag_send_ms_sum += dt_ms
            self.diag_send_ms_max = max(self.diag_send_ms_max, dt_ms)
            code = ret if isinstance(ret, int) else int(ret[0]) if isinstance(ret, (list, tuple)) and ret else 0
            if code != 0:
                self.diag_send_fail += 1
                self.diag_fail_streak += 1
                if self.diag_fail_streak >= 5:
                    self.safe_motion_mode = f"send_error_{code}"
                    self.running = False
            else:
                self.diag_fail_streak = 0

        def _return_home_if_needed(self) -> None:
            if not self.safe_motion_plan.return_home_on_exit:
                return
            try:
                self.arm.set_mode(0)
                self.arm.set_state(0)
                self.arm.set_servo_angle(
                    angle=list(self.safe_motion_plan.home_joints_deg),
                    speed=min(20.0, max(5.0, self.safe_motion_plan.speed_deg_s)),
                    mvacc=120,
                    is_radian=False,
                    wait=True,
                )
            except Exception as exc:
                print(f"\n[WARN] return-home failed: {exc}")

        def control_loop(self):
            plan = self.safe_motion_plan
            q_deg_traj = np.rad2deg(plan.q_traj_rad)
            ts = plan.timestamps - float(plan.timestamps[0])
            if not self._enter_servo_joint_mode():
                print("\n[ERROR] failed to enter xArm servo joint mode")
                self.running = False
                return
            self.safe_motion_mode = "running"
            try:
                while self.running and (plan.cycles == 0 or self.safe_motion_cycles_done < plan.cycles):
                    t0 = time.monotonic()
                    for idx, target_q_deg in enumerate(q_deg_traj):
                        if not self.running:
                            break
                        self.diag_ctrl_iter += 1
                        self.safe_motion_point_index = int(idx)
                        if self.saving_evt.is_set():
                            time.sleep(0.02)
                            continue
                        delay = float(ts[idx]) - (time.monotonic() - t0)
                        if delay > 0:
                            time.sleep(delay)
                        self._send_joint_target(target_q_deg)
                        self.last_velocity_cmd = [0.0] * 6
                    self.safe_motion_cycles_done += 1
                if plan.cycles > 0 and self.safe_motion_cycles_done >= plan.cycles:
                    self.running = False
            finally:
                self.safe_motion_mode = "returning_home" if plan.return_home_on_exit else "stopping"
                self._return_home_if_needed()
                self.safe_motion_mode = "stopped"

        def maybe_print_diag(self):
            super().maybe_print_diag()
            if self.last_diag is not None:
                self.last_diag["safe_motion_mode"] = self.safe_motion_mode
                self.last_diag["safe_motion_cycles_done"] = self.safe_motion_cycles_done
                self.last_diag["safe_motion_point_index"] = self.safe_motion_point_index
                self.publish_web_telemetry()

        def run(self):
            self.start_input()
            print(
                f"[RUN] safe workspace motion started: dashboard={'on' if self.web_dashboard else 'off'} "
                f"url=http://{self.web_host}:{self.web_port}/ cycles={self.safe_motion_plan.cycles or 'infinite'} "
                f"dataset={self.lerobot_dataset.root}"
            )
            self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
            self.control_thread.start()
            if self.show_video and self._get_display_camera() is not None:
                self.video_thread = threading.Thread(target=self.video_loop, daemon=True)
                self.video_thread.start()

            dt = 1.0 / self.rate_hz
            while self.running:
                self.diag_main_iter += 1
                gdir, gpos, end = self.consume_events()
                self.step_gripper(gdir)
                self.set_gripper_discrete(gpos)
                st = self.read_state()
                st["action_velocity_cmd"] = list(self.last_velocity_cmd)
                st["action_gripper"] = float(self.gripper_pos)
                self.capture_episode_observation(st)
                self.publish_web_telemetry()
                self.print_currents(st)
                self.maybe_print_diag()
                if end:
                    self.saving_evt.set()
                    self.stop_motion_now()
                    self.save_episode()
                    self.saving_evt.clear()
                time.sleep(dt)

            print("\n[STOP] shutting down safe workspace runner")
            if self.control_thread is not None:
                self.control_thread.join(timeout=10.0)
                if self.control_thread.is_alive():
                    print("[WARN] safe motion thread did not stop in time; sending stop command")
                    self.stop_motion_now()
            else:
                self.stop_motion_now()
            if self.video_thread is not None:
                self.video_thread.join(timeout=1.0)
            try:
                self.save_episode()
            finally:
                if self.lerobot_dataset is not None:
                    self.lerobot_dataset.finalize()
                    print("[INFO] LeRobot dataset finalized")
            if self.unclutter_proc is not None:
                try:
                    self.unclutter_proc.terminate()
                except Exception:
                    pass
            if self.camera is not None:
                self.camera.close()
            if self.base_camera is not None:
                self.base_camera.close()
            if self.target_camera is not None:
                self.target_camera.close()
            if self.dashboard_server is not None:
                self.dashboard_server.stop()
            if self.torque_estimator is not None:
                self.torque_estimator.close()
            self.arm.disconnect()

    return SafeWorkspaceMotionApp


def run(args: argparse.Namespace, config: dict, plan: SafeMotionPlan) -> None:
    from record.teleop import TeleopApp
    from xarm.core.config.x_config import XCONF

    connection = config.get("connection", {}) or {}
    robot_ip = str(connection.get("ip", "192.168.1.199"))
    robot_port = int(connection.get("robot_port", 502))
    report_normal = int(connection.get("report_port_normal", 30001))
    report_rich = int(connection.get("report_port_rich", 30002))
    report_real = int(connection.get("report_port_real", 30003))
    sdk_timeout_s = float(connection.get("sdk_timeout_s", 0.5))

    XCONF.SocketConf.TCP_CONTROL_PORT = robot_port
    XCONF.SocketConf.TCP_REPORT_NORM_PORT = report_normal
    XCONF.SocketConf.TCP_REPORT_RICH_PORT = report_rich
    XCONF.SocketConf.TCP_REPORT_REAL_PORT = report_real

    app_cls = _make_safe_app_class(TeleopApp)
    app = app_cls(
        robot_ip,
        args.rate_hz,
        args.control_hz,
        args.data_dir,
        args.camera_id,
        args.camera_dev,
        args.base_camera_id,
        args.base_camera_dev,
        args.target_camera_id,
        args.target_camera_dev,
        args.display_camera,
        (not args.allow_camera_fallback),
        args.show_video,
        False,
        sdk_timeout_s,
        0.0 if not args.show_video else 24.0,
        args.repo_id,
        args.task,
        args.vcodec,
        args.encoder_threads,
        args.encoder_queue_maxsize,
        None if args.no_torque_comp else args.torque_comp_model,
        args.web_dashboard,
        args.web_host,
        args.web_port,
        args.web_fps,
        (not args.no_start_reset),
        safe_motion_plan=plan,
        record_dataset=args.record_dataset,
    )

    def _handle(_sig, _frame):
        app.running = False

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)
    app.run()


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    config = load_config(args.config, robot=args.robot, overrides=_config_overrides(args))
    if args.no_torque_comp:
        args.torque_comp_model = None
    if args.torque_comp_model and not Path(args.torque_comp_model).expanduser().is_file():
        raise SystemExit(f"[ERROR] torque compensation model not found: {args.torque_comp_model}")
    plan = build_motion_plan(args, config)
    _print_plan(plan, config, args)
    if args.self_check:
        print("SELF_CHECK_OK")
        return 0
    run(args, config, plan)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
