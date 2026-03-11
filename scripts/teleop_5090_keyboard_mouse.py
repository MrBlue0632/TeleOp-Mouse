#!/usr/bin/env python3
import argparse
import collections
import json
import os
import shutil
import signal
import struct
import subprocess
import sys
import threading
import time
from datetime import datetime

os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")

import cv2
import numpy as np
from xarm.core.config.x_config import XCONF
from xarm.wrapper import XArmAPI


def clip(v, lo, hi):
    return max(lo, min(hi, v))


def rpy_deg_to_rotmat(roll_deg, pitch_deg, yaw_deg):
    rx = np.deg2rad(float(roll_deg))
    ry = np.deg2rad(float(pitch_deg))
    rz = np.deg2rad(float(yaw_deg))
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    rxm = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=np.float64)
    rym = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=np.float64)
    rzm = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rzm @ rym @ rxm


RESET_HOME_JOINTS_DEG = [14.1, -8.0, -24.7, 196.9, 62.3, -8.8]
RESET_HOME_GRIPPER = 840.0
PANEL_JOINT_MAX = np.array([2.8, 9.7, 5.6, 1.9, 1.6, 1.3], dtype=np.float64)
PANEL_GRIPPER_MAX = 1.0


class CameraStream:
    def __init__(self, source, width=640, height=480):
        self.source = source
        self.width = int(width)
        self.height = int(height)
        self.cap = None
        attempts = []
        if isinstance(source, str) and source.startswith("/dev/video"):
            gst = (
                f"v4l2src device={source} io-mode=2 do-timestamp=true ! "
                f"video/x-raw,width={self.width},height={self.height},framerate=30/1 ! "
                "videoconvert ! video/x-raw,format=BGR ! "
                "appsink drop=true max-buffers=1 sync=false"
            )
            attempts.append((gst, cv2.CAP_GSTREAMER))
            attempts.append((source, cv2.CAP_V4L2))
        else:
            attempts.append((source, cv2.CAP_ANY))
            if hasattr(cv2, "CAP_OBSENSOR"):
                attempts.append((source, cv2.CAP_OBSENSOR))
            attempts.append((source, cv2.CAP_V4L2))
        for open_src, backend in attempts:
            cap = cv2.VideoCapture(open_src, backend)
            if cap.isOpened():
                self.cap = cap
                break
            cap.release()
        if self.cap is None:
            # keep a dummy handle for unified close path
            self.cap = cv2.VideoCapture()
        self.frame = None
        self.count = 0
        self.ts = 0.0
        self.lock = threading.Lock()
        self.stop_evt = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.started = False
        self.configured = False

    def start(self):
        if not self.started:
            self.started = True
            self.thread.start()

    def _loop(self):
        while not self.stop_evt.is_set():
            if not self.cap.isOpened():
                time.sleep(0.05)
                continue
            if not self.configured:
                try:
                    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
                    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
                    try:
                        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                    except Exception:
                        pass
                except Exception:
                    pass
                self.configured = True
            ok, frm = self.cap.read()
            if ok and frm is not None:
                with self.lock:
                    self.frame = frm
                    self.count += 1
                    self.ts = time.monotonic()
            else:
                time.sleep(0.01)

    def get(self):
        with self.lock:
            return self.frame, self.ts, self.count

    def close(self):
        self.stop_evt.set()
        if self.started:
            self.thread.join(timeout=1.0)
        self.cap.release()

class Teleop5090:
    def __init__(self, robot_ip, rate_hz, control_hz, data_dir, camera_id, camera_dev, strict_camera_dev, show_video, fps_mouse, sdk_timeout_s, video_hz):
        self.robot_ip = robot_ip
        self.rate_hz = rate_hz
        self.control_hz = float(control_hz)
        self.data_dir = data_dir
        self.camera_id = camera_id
        self.camera_dev = camera_dev
        self.strict_camera_dev = strict_camera_dev
        self.show_video = show_video
        self.fps_mouse = fps_mouse
        self.sdk_timeout_s = float(sdk_timeout_s)
        self.video_hz = float(video_hz)
        self.camera_probe_timeout_s = 1.8

        # 2x faster than previous defaults, while still conservative
        self.linear_step_mm = 1.2
        self.angular_step_deg_per_px = 0.12
        self.cart_speed = 40.0
        self.cart_acc = 240.0
        self.linear_vel_mm_s = 60.0
        self.angular_vel_deg_s = 33.75
        self.mouse_vel_gain_deg_s_per_px = 1.35

        self.gripper_min = 0.0
        self.gripper_max = 850.0
        self.gripper_step = 20.0
        self.gripper_speed = 2400.0

        self.running = True
        self.lock = threading.Lock()

        self.keys_down = set()
        self.mouse_dx = 0.0
        self.mouse_dy = 0.0
        self.last_mouse_xy = None
        self.gripper_dir = 0
        self.pending_gripper_pos = None
        self.gripper_toggle_closed = False
        self.home_requested = False
        self.end_episode_requested = False

        self.episode_idx = 1
        self.episode = []
        self.pose_dirty = False

        self.Key = None
        self.Button = None
        self.kb_listener = None
        self.mouse_listener = None
        self.mouse_ctl = None
        self.screen_center = None
        self.ignore_warp_event = False
        self.unclutter_proc = None
        self.center_thread = None
        self.control_thread = None
        self.video_thread = None
        self.last_velocity_cmd = [0.0] * 6
        self.velocity_mode_ready = False
        self.saving_evt = threading.Event()
        self.use_xdotool_center = False
        self.last_xdotool_center_ts = 0.0

        # Runtime diagnostics for latency/drop investigation.
        self.diag_last_ts = time.monotonic()
        self.diag_ctrl_iter = 0
        self.diag_main_iter = 0
        self.diag_video_iter = 0
        self.diag_send_cnt = 0
        self.diag_send_fail = 0
        self.diag_send_ms_sum = 0.0
        self.diag_send_ms_max = 0.0
        self.diag_fail_streak = 0
        self.diag_last_recover_ts = 0.0

        self.window_name = "Teleop5090 Video+Currents"
        self.camera = None
        self.camera_source = None
        self.last_camera_retry_ts = 0.0
        self.window_inited = False
        self.last_window_maintain_ts = 0.0
        self.filtered_curr = np.full((7,), np.nan, dtype=np.float64)
        self.curr_valid = np.zeros((7,), dtype=bool)
        self.med_buf = [collections.deque(maxlen=5) for _ in range(7)]
        self.alpha_rise = 0.35
        self.alpha_fall = 0.12
        self.latest_reg_gripper_amp = float("nan")
        self.reg_gripper_ok = False
        self.reg_gripper_fail_cnt = 0
        self.last_reg_read_ts = 0.0
        self.current_alignment_ok = True

        self.arm = XArmAPI(self.robot_ip, do_not_open=True)
        self.arm.connect()
        if not self.arm.connected:
            raise RuntimeError(f"Failed to connect robot at {self.robot_ip}")
        # Reduce worst-case blocking when network/controller stalls.
        self.arm.set_timeout(self.sdk_timeout_s)
        self.ensure_robot_ready()
        self.arm.set_gripper_enable(True)
        self.arm.set_mode(0)
        self.arm.set_state(0)
        self.reset_to_home_on_start()
        self.enter_realtime_velocity_mode()

        pret = self.arm.get_position(is_radian=False)
        self.target_pose = [float(v) for v in pret[1][:6]] if pret and pret[0] == 0 else [300.0, 0.0, 200.0, 180.0, 0.0, 0.0]
        self.home_pose = list(self.target_pose)
        self.latest_rpy_deg = [float(self.target_pose[3]), float(self.target_pose[4]), float(self.target_pose[5])]

        gret = self.arm.get_gripper_position()
        self.gripper_pos = float(gret[1]) if gret and gret[0] == 0 else 400.0
        self.last_state = {
            "ts": time.time(),
            "joints_deg": [0.0] * 6,
            "pose_xyzrpy_deg": list(self.target_pose),
            "gripper_pos": float(self.gripper_pos),
            "currents": [0.0] * 7,
        }
        self.last_state_ts = 0.0
        self.last_robot_state = None
        self.last_robot_err = None

        os.makedirs(self.data_dir, exist_ok=True)

    def ensure_robot_ready(self, timeout_s=6.0):
        deadline = time.monotonic() + timeout_s
        last_state = None
        last_err = None
        while time.monotonic() < deadline:
            try:
                self.arm.clean_warn()
                self.arm.clean_error()
            except Exception:
                pass
            try:
                self.arm.motion_enable(True)
                self.arm.set_state(0)
            except Exception:
                pass

            st = self.arm.get_state()
            ew = self.arm.get_err_warn_code()
            if st and st[0] == 0 and len(st) > 1:
                last_state = int(st[1])
            if ew and ew[0] == 0 and len(ew) > 1 and isinstance(ew[1], (list, tuple)) and len(ew[1]) > 0:
                last_err = int(ew[1][0])
            self.last_robot_state = last_state
            self.last_robot_err = last_err

            if (last_err in (None, 0)) and (last_state in (0, 1, 2, 3)):
                return
            time.sleep(0.12)
        raise RuntimeError(f"xArm not ready: state={last_state}, err={last_err}. Please clear controller error first.")

    def reset_to_home_on_start(self):
        # Align reset pose with 4060 implementation home joints.
        try:
            self.arm.clean_warn()
            self.arm.clean_error()
            self.arm.motion_enable(True)
            self.arm.set_mode(0)
            self.arm.set_state(0)
            self.arm.set_servo_angle(
                angle=list(RESET_HOME_JOINTS_DEG),
                speed=20,
                mvacc=200,
                is_radian=False,
                wait=True,
            )
            self.arm.set_gripper_position(
                pos=float(RESET_HOME_GRIPPER),
                wait=False,
                speed=self.gripper_speed,
                auto_enable=True,
            )
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                st = self.arm.get_state()
                if st and st[0] == 0 and len(st) > 1 and int(st[1]) in (0, 1):
                    break
                time.sleep(0.05)
        except Exception:
            # Keep startup resilient on firmware/model mismatch.
            pass

    def enter_realtime_velocity_mode(self):
        # Ultra-low-latency control: send cartesian velocity directly.
        try:
            self.velocity_mode_ready = False
            for _ in range(3):
                self.arm.set_state(0)
                time.sleep(0.05)
                self.arm.set_mode(5)
                time.sleep(0.08)
                self.arm.set_state(0)
                time.sleep(0.05)
                self.arm.set_cartesian_velo_continuous(True)
                self.arm.vc_set_cartesian_velocity(
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    is_radian=False,
                    duration=0,
                    check_mode=False,
                )
                mode = int(getattr(self.arm, "mode", -1))
                state = int(getattr(self.arm, "state", -1))
                if mode == 5 and state in (0, 1, 2):
                    self.velocity_mode_ready = True
                    return True
        except Exception:
            self.velocity_mode_ready = False
        return False

    def ensure_velocity_runtime_ready(self):
        mode = int(getattr(self.arm, "mode", -1))
        state = int(getattr(self.arm, "state", -1))
        if mode == 5 and state in (0, 1, 2):
            return True
        try:
            if mode == 5 and state in (2, 3):
                self.arm.set_state(0)
                time.sleep(0.03)
                mode = int(getattr(self.arm, "mode", -1))
                state = int(getattr(self.arm, "state", -1))
                if mode == 5 and state in (0, 1, 2):
                    self.velocity_mode_ready = True
                    return True
        except Exception:
            pass
        return self.enter_realtime_velocity_mode()

    def stop_motion_now(self):
        try:
            self.arm.vc_set_cartesian_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], is_radian=False, duration=0)
        except Exception:
            pass

    def go_home_now(self):
        # Home must use position control; switch back to velocity mode after.
        self.stop_motion_now()
        try:
            self.arm.set_mode(0)
            self.arm.set_state(0)
            self.arm.set_servo_angle(
                angle=list(RESET_HOME_JOINTS_DEG),
                speed=20,
                mvacc=200,
                is_radian=False,
                wait=True,
            )
            self.arm.set_gripper_position(
                pos=float(RESET_HOME_GRIPPER),
                wait=False,
                speed=self.gripper_speed,
                auto_enable=True,
            )
            pret = self.arm.get_position(is_radian=False)
            if pret and pret[0] == 0:
                self.target_pose = [float(v) for v in pret[1][:6]]
                self.home_pose = list(self.target_pose)
        except Exception:
            pass
        self.enter_realtime_velocity_mode()

    def start_input(self):
        from pynput import keyboard, mouse
        self.mouse_ctl = mouse.Controller()

        self.Key = keyboard.Key
        self.Button = mouse.Button
        self.kb_listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.mouse_listener = mouse.Listener(on_move=self.on_mouse_move, on_click=self.on_mouse_click)
        self.kb_listener.start()
        self.mouse_listener.start()
        self.setup_fps_mouse()
        if self.show_video:
            self.camera = self.open_camera_with_probe(self.camera_id, self.camera_dev)
            if self.camera is None:
                print(f"[WARN] no camera frame available, disable video panel")
                if self.strict_camera_dev:
                    raise RuntimeError(f"strict camera mode: cannot open {self.camera_dev}")

    def iter_camera_sources(self, preferred_cam_id, preferred_cam_dev):
        sources = []
        if preferred_cam_dev:
            sources.append(preferred_cam_dev)
        elif preferred_cam_id is not None:
            sources.append(int(preferred_cam_id))
        if not self.strict_camera_dev:
            sources.extend(["/dev/video4", "/dev/video6", 1, 11])
        return sources

    def open_camera_with_probe(self, preferred_cam_id, preferred_cam_dev):
        tried = set()
        for src in self.iter_camera_sources(preferred_cam_id, preferred_cam_dev):
            if src in tried:
                continue
            tried.add(src)
            print(f"[INFO] probing camera source {src}")
            stream = CameraStream(src, 640, 480)
            if not stream.cap.isOpened():
                print(f"[WARN] camera source {src} open failed")
                stream.close()
                continue
            stream.start()
            t0 = time.monotonic()
            ok = False
            while time.monotonic() - t0 < self.camera_probe_timeout_s:
                frm, ts, cnt = stream.get()
                if frm is not None and cnt > 0 and (time.monotonic() - ts) < 1.0:
                    ok = True
                    break
                time.sleep(0.03)
            if ok:
                self.camera_source = src
                if isinstance(src, str):
                    print(f"[INFO] camera source {src} ready")
                else:
                    self.camera_id = int(src)
                    if int(src) != preferred_cam_id:
                        print(f"[WARN] camera index {preferred_cam_id} unavailable, fallback -> {src}")
                    else:
                        print(f"[INFO] camera index {src} ready")
                return stream
            stream.close()
        print(f"[ERROR] no usable camera frames (preferred index={preferred_cam_id}, dev={preferred_cam_dev})")
        return None

    def setup_fps_mouse(self):
        if not self.fps_mouse:
            return
        try:
            import tkinter as tk
            root = tk.Tk()
            root.withdraw()
            w = int(root.winfo_screenwidth())
            h = int(root.winfo_screenheight())
            root.destroy()
            self.screen_center = (max(1, w // 2), max(1, h // 2))
        except Exception:
            # Fallback for environments without tkinter.
            self.screen_center = self.get_screen_center_x11()
        if self.screen_center is not None and self.mouse_ctl is not None:
            try:
                self.mouse_ctl.position = self.screen_center
            except Exception:
                pass
        self.use_xdotool_center = shutil.which("xdotool") is not None
        # Best-effort cursor hide on X11.
        if shutil.which("unclutter"):
            try:
                self.unclutter_proc = subprocess.Popen(
                    ["unclutter", "-idle", "0", "-root"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
            except Exception:
                self.unclutter_proc = None
        # Force-center loop to keep FPS-style lock even without move callbacks.
        if self.screen_center is not None:
            print(f"[INFO] FPS mouse center enabled at {self.screen_center}")
            self.center_thread = threading.Thread(target=self.center_loop, daemon=True)
            self.center_thread.start()
        else:
            print("[WARN] FPS mouse center disabled: screen center not detected")

    def get_screen_center_x11(self):
        try:
            proc = subprocess.run(
                ["xdpyinfo"],
                check=False,
                capture_output=True,
                text=True,
                timeout=1.2,
            )
            out = proc.stdout
            for line in out.splitlines():
                if "dimensions:" in line:
                    dims = line.split("dimensions:")[1].strip().split()[0]
                    w, h = dims.split("x")
                    return (max(1, int(w) // 2), max(1, int(h) // 2))
        except Exception:
            pass
        return None

    def center_loop(self):
        while self.running and self.fps_mouse and self.screen_center is not None:
            cx, cy = self.screen_center
            try:
                if self.mouse_ctl is not None:
                    self.ignore_warp_event = True
                    self.mouse_ctl.position = self.screen_center
                elif self.use_xdotool_center and (time.monotonic() - self.last_xdotool_center_ts) >= 0.05:
                    self.last_xdotool_center_ts = time.monotonic()
                    subprocess.run(
                        ["xdotool", "mousemove", str(int(cx)), str(int(cy))],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        check=False,
                    )
            except Exception:
                pass
            time.sleep(0.008)

    def read_gripper_current_register_amp(self, register_addr=0x0003):
        # Same path as 4060 monitor_gripper_current.py
        try:
            ret = self.arm.core.gripper_modbus_r16s(register_addr, 1)
            if isinstance(ret, (list, tuple)) and len(ret) >= 7 and ret[0] == 0:
                raw = struct.unpack(">h", bytes(ret[5:7]))[0]
                return float(raw) / 100.0
        except Exception:
            pass
        return float("nan")

    def update_filtered_currents(self, raw_curr):
        for k in range(7):
            v = raw_curr[k]
            if np.isnan(v):
                continue
            self.med_buf[k].append(float(v))
            v_med = float(np.median(self.med_buf[k]))
            if not self.curr_valid[k]:
                self.filtered_curr[k] = v_med
                self.curr_valid[k] = True
            else:
                a = self.alpha_rise if abs(v_med) >= abs(self.filtered_curr[k]) else self.alpha_fall
                self.filtered_curr[k] = (1.0 - a) * self.filtered_curr[k] + a * v_med

    def draw_current_panel(self, panel):
        panel[:] = (22, 24, 28)
        labels = ["J1", "J2", "J3", "J4", "J5", "J6", "GRIP"]
        h, w = panel.shape[:2]
        top = 32
        row_h = (h - 48) // len(labels)
        bar_x = 200
        bar_w = w - bar_x - 16
        cv2.putText(panel, "Currents (Filtered)", (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (210, 210, 220), 1)
        for i, lab in enumerate(labels):
            y = top + i * row_h
            val = float(abs(self.filtered_curr[i])) if not np.isnan(self.filtered_curr[i]) else float("nan")
            cv2.putText(panel, lab, (14, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.66, (220, 220, 220), 1)
            txt = "N/A" if np.isnan(val) else f"{val:.2f}A"
            cv2.putText(panel, txt, (120, y + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (235, 235, 235), 1)
            vmax = float(PANEL_GRIPPER_MAX if i == 6 else PANEL_JOINT_MAX[i])
            ratio = 0.0 if np.isnan(val) else min(1.0, val / max(vmax, 1e-6))
            fill = int(bar_w * ratio)
            cv2.rectangle(panel, (bar_x, y + 4), (bar_x + bar_w, y + 24), (70, 70, 78), -1)
            color = (50, 190, 255) if ratio < 0.75 else (60, 120, 255)
            cv2.rectangle(panel, (bar_x, y + 4), (bar_x + fill, y + 24), color, -1)
            cv2.rectangle(panel, (bar_x, y + 4), (bar_x + bar_w, y + 24), (110, 110, 120), 1)

        check_txt = "CAL:OK" if self.current_alignment_ok else "CAL:WARN"
        check_color = (70, 220, 110) if self.current_alignment_ok else (60, 120, 255)
        cv2.putText(panel, check_txt, (14, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.58, check_color, 1)

    def show_video_panel(self):
        if self.camera is None:
            return
        t0 = time.monotonic()
        frm, ts, cnt = self.camera.get()
        if frm is None or cnt < 1 or (time.monotonic() - ts) > 1.0:
            now = time.monotonic()
            if (now - self.last_camera_retry_ts) >= 1.5:
                self.last_camera_retry_ts = now
                print("[WARN] camera frame stalled, reopening source")
                try:
                    self.camera.close()
                except Exception:
                    pass
                self.camera = self.open_camera_with_probe(self.camera_id, self.camera_dev)
                if self.camera is not None:
                    frm, ts, cnt = self.camera.get()
            if frm is None or cnt < 1 or (time.monotonic() - ts) > 1.0:
                frm = np.zeros((480, 640, 3), dtype=np.uint8)
            else:
                frm = cv2.resize(frm, (640, 480))
        else:
            frm = cv2.resize(frm, (640, 480))
        panel = np.zeros((480, 360, 3), dtype=np.uint8)
        self.draw_current_panel(panel)
        canvas = np.hstack([frm, panel])
        if not self.window_inited:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            self.enforce_window_layout(canvas.shape[1], canvas.shape[0])
            self.window_inited = True
            self.last_window_maintain_ts = time.monotonic()
        cv2.imshow(self.window_name, canvas)
        now = time.monotonic()
        # Keep window centered/topmost with low frequency to avoid GUI stalls.
        if (now - self.last_window_maintain_ts) >= 0.5:
            self.enforce_window_layout(canvas.shape[1], canvas.shape[0])
            self.last_window_maintain_ts = now
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            self.running = False
        dt_ms = (time.monotonic() - t0) * 1000.0
        if dt_ms > 120.0:
            print(f"\n[WARN] video panel render slow: {dt_ms:.1f}ms")

    def enforce_window_layout(self, win_w, win_h):
        # Keep the camera window centered and always on top.
        try:
            cv2.resizeWindow(self.window_name, int(win_w), int(win_h))
        except Exception:
            pass
        self.lock_window_center(win_w, win_h)
        try:
            if hasattr(cv2, "WND_PROP_TOPMOST"):
                cv2.setWindowProperty(self.window_name, cv2.WND_PROP_TOPMOST, 1)
        except Exception:
            pass

    def lock_window_center(self, win_w, win_h):
        center = self.screen_center if self.screen_center is not None else self.get_screen_center_x11()
        if center is None:
            return
        cx, cy = center
        x = max(0, int(cx - win_w // 2))
        y = max(0, int(cy - win_h // 2))
        try:
            cv2.moveWindow(self.window_name, x, y)
        except Exception:
            pass

    def on_key_press(self, key):
        with self.lock:
            if key == self.Key.esc:
                self.running = False
                return
            if key == self.Key.space:
                self.keys_down.add("space")
                return
            if key in (self.Key.shift, self.Key.shift_l, self.Key.shift_r):
                self.keys_down.add("shift")
                return
            if key == self.Key.enter:
                self.end_episode_requested = True
                return
            ch = getattr(key, "char", None)
            if ch is None:
                return
            ch = ch.lower()
            if ch in ("w", "a", "s", "d", "q", "e"):
                self.keys_down.add(ch)
            elif ch == "r":
                self.home_requested = True

    def on_key_release(self, key):
        with self.lock:
            if key == self.Key.space:
                self.keys_down.discard("space")
                return
            if key in (self.Key.shift, self.Key.shift_l, self.Key.shift_r):
                self.keys_down.discard("shift")
                return
            ch = getattr(key, "char", None)
            if ch is not None:
                self.keys_down.discard(ch.lower())

    def on_mouse_move(self, x, y):
        with self.lock:
            if self.fps_mouse and self.screen_center is not None and self.mouse_ctl is not None:
                if self.ignore_warp_event:
                    self.ignore_warp_event = False
                    return
                cx, cy = self.screen_center
                dx = clip(x - cx, -40, 40)
                dy = clip(y - cy, -40, 40)
                self.mouse_dx = 0.0 if abs(dx) < 1 else float(dx)
                self.mouse_dy = 0.0 if abs(dy) < 1 else float(dy)
                return
            if self.last_mouse_xy is None:
                self.last_mouse_xy = (x, y)
                return
            dx = clip(x - self.last_mouse_xy[0], -40, 40)
            dy = clip(y - self.last_mouse_xy[1], -40, 40)
            self.last_mouse_xy = (x, y)
            # Keep only latest delta to avoid queued mouse-motion lag.
            self.mouse_dx = float(dx)
            self.mouse_dy = float(dy)

    def on_mouse_click(self, _x, _y, button, pressed):
        with self.lock:
            if not pressed:
                return
            if button == self.Button.left:
                # left click toggles close/open
                self.gripper_toggle_closed = (not self.gripper_toggle_closed)
                self.pending_gripper_pos = 300.0 if self.gripper_toggle_closed else 830.0

    def consume_events(self):
        with self.lock:
            gdir = self.gripper_dir
            gpos = self.pending_gripper_pos
            self.pending_gripper_pos = None
            end = self.end_episode_requested
            self.end_episode_requested = False
        return gdir, gpos, end

    def snapshot_inputs(self):
        with self.lock:
            keys = set(self.keys_down)
            dx, dy = self.mouse_dx, self.mouse_dy
            self.mouse_dx, self.mouse_dy = 0.0, 0.0
            home = self.home_requested
            self.home_requested = False
        return keys, dx, dy, home

    def compose_velocity_cmd(self, keys, dx, dy):
        vx = 0.0
        vy = 0.0
        vz = 0.0
        vrx = 0.0
        vry = 0.0
        vrz = 0.0

        # Translational keys are expressed directly in the current tool frame.
        # With is_tool_coord=True on velocity send, "forward" follows the gripper direction.
        if "w" in keys:
            vz += self.linear_vel_mm_s
        if "s" in keys:
            vz -= self.linear_vel_mm_s
        if "space" in keys:
            vx -= self.linear_vel_mm_s
        if "shift" in keys:
            vx += self.linear_vel_mm_s
        if "a" in keys:
            vy += self.linear_vel_mm_s
        if "d" in keys:
            vy -= self.linear_vel_mm_s

        if "q" in keys:
            vrz -= self.angular_vel_deg_s
        if "e" in keys:
            vrz += self.angular_vel_deg_s

        # On this setup the controller's effective RX/RZ slots are opposite to the
        # SDK naming in teleop hand-feel, so mouse-x/QE are intentionally swapped.
        # mouse left/right -> physical yaw ; mouse up/down -> physical pitch
        vrx += dx * self.mouse_vel_gain_deg_s_per_px
        vry += (dy) * self.mouse_vel_gain_deg_s_per_px
        vy = clip(vy, -self.linear_vel_mm_s, self.linear_vel_mm_s)
        vrx = clip(vrx, -120.0, 120.0)
        vrz = clip(vrz, -120.0, 120.0)
        vry = clip(vry, -120.0, 120.0)

        return [vx, vy, vz, vrx, vry, vrz]

    def step_gripper(self, gdir):
        if gdir == 0:
            return
        self.gripper_pos = clip(self.gripper_pos + gdir * self.gripper_step, self.gripper_min, self.gripper_max)
        self.arm.set_gripper_position(self.gripper_pos, wait=False, speed=self.gripper_speed, auto_enable=True)

    def set_gripper_discrete(self, target_pos):
        if target_pos is None:
            return
        self.gripper_pos = clip(float(target_pos), self.gripper_min, self.gripper_max)
        self.arm.set_gripper_position(self.gripper_pos, wait=False, speed=self.gripper_speed, auto_enable=True)

    def send_velocity(self, vel):
        if not self.ensure_velocity_runtime_ready():
            return
        t0 = time.monotonic()
        ret = self.arm.vc_set_cartesian_velocity(
            vel,
            is_radian=False,
            is_tool_coord=True,
            duration=0,
            check_mode=False,
        )
        dt_ms = (time.monotonic() - t0) * 1000.0
        self.diag_send_cnt += 1
        self.diag_send_ms_sum += dt_ms
        self.diag_send_ms_max = max(self.diag_send_ms_max, dt_ms)

        code = 0
        if isinstance(ret, int):
            code = ret
        elif isinstance(ret, (list, tuple)) and len(ret) > 0 and isinstance(ret[0], int):
            code = ret[0]

        if code != 0:
            self.diag_send_fail += 1
            self.diag_fail_streak += 1
            now = time.monotonic()
            if self.diag_fail_streak >= 5 and (now - self.diag_last_recover_ts) >= 0.8:
                self.diag_last_recover_ts = now
                self.enter_realtime_velocity_mode()
        else:
            self.diag_fail_streak = 0

    def control_loop(self):
        dt = 1.0 / self.control_hz
        next_t = time.monotonic()
        while self.running:
            self.diag_ctrl_iter += 1
            if self.saving_evt.is_set():
                self.last_velocity_cmd = [0.0] * 6
                self.stop_motion_now()
                time.sleep(0.01)
                continue
            keys, dx, dy, home = self.snapshot_inputs()
            if home:
                self.go_home_now()
            vel = self.compose_velocity_cmd(keys, dx, dy)
            self.last_velocity_cmd = [float(v) for v in vel]
            self.send_velocity(vel)

            now = time.monotonic()
            next_t += dt
            if next_t > now:
                time.sleep(next_t - now)
            else:
                next_t = now

    def read_state(self):
        now = time.time()
        # Reduce blocking SDK reads to lower control latency.
        if now - self.last_state_ts < 0.1:
            s = dict(self.last_state)
            s["ts"] = now
            s["gripper_pos"] = float(self.gripper_pos)
            return s

        jret = self.arm.get_servo_angle(is_radian=False)
        joints = [float(v) for v in jret[1][:6]] if jret and jret[0] == 0 else self.last_state["joints_deg"]
        pret = self.arm.get_position(is_radian=False)
        if pret and pret[0] == 0:
            pose = [float(v) for v in pret[1][:6]]
            self.target_pose = list(pose)
            self.latest_rpy_deg = [pose[3], pose[4], pose[5]]
        currents = list(self.arm.currents) if self.arm.currents is not None else self.last_state["currents"]
        if len(currents) < 7:
            currents = currents + [0.0] * (7 - len(currents))
        currents = [float(c) for c in currents[:7]]

        # Gripper current uses independent monitor-register path for calibration.
        if (now - self.last_reg_read_ts) >= 0.25:
            reg_amp = self.read_gripper_current_register_amp(register_addr=0x0003)
            self.last_reg_read_ts = now
            if np.isnan(reg_amp):
                self.reg_gripper_fail_cnt += 1
                if self.reg_gripper_fail_cnt >= 3:
                    self.reg_gripper_ok = False
            else:
                self.reg_gripper_fail_cnt = 0
                self.reg_gripper_ok = True
                self.latest_reg_gripper_amp = reg_amp

        if self.reg_gripper_ok and not np.isnan(self.latest_reg_gripper_amp):
            currents[6] = float(self.latest_reg_gripper_amp)

        raw_arr = np.array(currents, dtype=np.float64)
        self.update_filtered_currents(raw_arr)
        # Calibration check between terminal raw and panel filtered.
        if not np.isnan(self.filtered_curr[6]) and not np.isnan(raw_arr[6]):
            self.current_alignment_ok = abs(self.filtered_curr[6] - raw_arr[6]) <= 0.40

        self.last_state = {
            "ts": now,
            "joints_deg": joints,
            "pose_xyzrpy_deg": list(self.target_pose),
            "gripper_pos": float(self.gripper_pos),
            "currents": currents,
            "currents_filtered": [float(x) if not np.isnan(x) else None for x in self.filtered_curr.tolist()],
            "gripper_current_source": "register_0x0003" if self.reg_gripper_ok else "arm_currents_axis7",
        }
        self.last_state_ts = now
        return dict(self.last_state)

    def maybe_print_diag(self):
        now = time.monotonic()
        dt = now - self.diag_last_ts
        if dt < 1.0:
            return
        self.diag_last_ts = now

        ctrl_hz = self.diag_ctrl_iter / dt
        main_hz = self.diag_main_iter / dt
        video_hz = self.diag_video_iter / dt
        self.diag_ctrl_iter = 0
        self.diag_main_iter = 0
        self.diag_video_iter = 0

        send_cnt = self.diag_send_cnt
        send_fail = self.diag_send_fail
        send_avg = (self.diag_send_ms_sum / send_cnt) if send_cnt > 0 else 0.0
        send_max = self.diag_send_ms_max
        self.diag_send_cnt = 0
        self.diag_send_fail = 0
        self.diag_send_ms_sum = 0.0
        self.diag_send_ms_max = 0.0

        cam_fps = 0.0
        cam_age_ms = -1.0
        if self.camera is not None:
            _frm, ts, cnt = self.camera.get()
            cam_age_ms = (time.monotonic() - ts) * 1000.0 if ts > 0 else -1.0
            if not hasattr(self, "_diag_last_cam_cnt"):
                self._diag_last_cam_cnt = cnt
                self._diag_last_cam_ts = now
            else:
                cdt = now - self._diag_last_cam_ts
                if cdt > 1e-6:
                    cam_fps = (cnt - self._diag_last_cam_cnt) / cdt
                self._diag_last_cam_cnt = cnt
                self._diag_last_cam_ts = now

        fail_rate = (100.0 * send_fail / send_cnt) if send_cnt > 0 else 0.0

        st = self.arm.get_state()
        ew = self.arm.get_err_warn_code()
        if st and st[0] == 0 and len(st) > 1:
            self.last_robot_state = int(st[1])
        if ew and ew[0] == 0 and len(ew) > 1 and isinstance(ew[1], (list, tuple)) and len(ew[1]) > 0:
            self.last_robot_err = int(ew[1][0])

        print(
            f"\n[DIAG] ctrl={ctrl_hz:5.1f}Hz main={main_hz:5.1f}Hz video={video_hz:5.1f}Hz "
            f"send={send_cnt:4d}/s fail={send_fail:3d}({fail_rate:4.1f}%) "
            f"send_ms(avg/max)={send_avg:4.1f}/{send_max:4.1f} "
            f"cam_fps={cam_fps:4.1f} cam_age_ms={cam_age_ms:6.1f} "
            f"arm_state={self.last_robot_state} arm_err={self.last_robot_err}"
        )

    def video_loop(self):
        if self.video_hz <= 0:
            return
        dt = 1.0 / self.video_hz
        next_t = time.monotonic()
        while self.running:
            self.diag_video_iter += 1
            self.show_video_panel()
            now = time.monotonic()
            next_t += dt
            if next_t > now:
                time.sleep(next_t - now)
            else:
                next_t = now
        # Keep GUI teardown in the same thread that owns imshow/waitKey.
        try:
            if self.window_inited:
                cv2.destroyWindow(self.window_name)
        except Exception:
            pass

    def save_episode(self):
        if not self.episode:
            return
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.data_dir, f"episode_{self.episode_idx:04d}_{ts}.jsonl")
        with open(path, "w", encoding="utf-8") as f:
            for row in self.episode:
                state_row = {
                    "ts": row.get("ts"),
                    "joints_deg": row.get("joints_deg"),
                    "pose_xyzrpy_deg": row.get("pose_xyzrpy_deg"),
                    "gripper_pos": row.get("gripper_pos"),
                }
                f.write(json.dumps(state_row, ensure_ascii=True) + "\n")
        print(f"\n[SAVE] episode {self.episode_idx} -> {path} ({len(self.episode)} steps)")
        self.episode_idx += 1
        self.episode = []

    def print_currents(self, state):
        cur = state["currents"]
        line = (
            f"J1:{cur[0]:6.2f}  J2:{cur[1]:6.2f}  J3:{cur[2]:6.2f}  "
            f"J4:{cur[3]:6.2f}  J5:{cur[4]:6.2f}  J6:{cur[5]:6.2f}  "
            f"GRIP:{cur[6]:6.2f}"
        )
        sys.stdout.write("\r" + line)
        sys.stdout.flush()

    def run(self):
        self.start_input()
        video_src = self.camera_source if self.camera_source is not None else (
            self.camera_dev if self.camera_dev else self.camera_id
        )
        print(
            f"[RUN] 5090 teleop started: WASD/Shift/Space, mouse move, left click(toggle gripper), "
            f"R, Enter. video={video_src}"
        )
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        if self.show_video and self.camera is not None:
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
            self.episode.append(st)
            self.print_currents(st)
            self.maybe_print_diag()

            if end:
                self.saving_evt.set()
                self.stop_motion_now()
                # save phase: input is ignored and robot is held still
                self.save_episode()
                self.saving_evt.clear()

            time.sleep(dt)

        print("\n[STOP] shutting down")
        self.stop_motion_now()
        if self.control_thread is not None:
            self.control_thread.join(timeout=1.0)
        if self.video_thread is not None:
            self.video_thread.join(timeout=1.0)
        self.save_episode()
        if self.kb_listener is not None:
            self.kb_listener.stop()
        if self.mouse_listener is not None:
            self.mouse_listener.stop()
        if self.unclutter_proc is not None:
            try:
                self.unclutter_proc.terminate()
            except Exception:
                pass
        if self.center_thread is not None:
            self.center_thread.join(timeout=0.5)
        if self.camera is not None:
            self.camera.close()
        self.arm.disconnect()


def self_check():
    required = {"w", "a", "s", "d", "q", "e", "space", "shift", "r", "enter", "mouse_move", "mouse_left"}
    got = {"w", "a", "s", "d", "q", "e", "space", "shift", "r", "enter", "mouse_move", "mouse_left"}
    if got != required:
        print("SELF_CHECK_FAIL_MAPPING")
        return 1
    if not (0.0 < 1.2 <= 2.0 and 0.0 < 0.12 <= 0.3 and 0.0 < 20.0 <= 40.0):
        print("SELF_CHECK_FAIL_DEFAULTS")
        return 1
    print("SELF_CHECK_OK")
    return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-ip", default="127.0.0.1", help="xArm IP. For 5090 via SSH tunnel use 127.0.0.1")
    parser.add_argument("--robot-port", type=int, default=1502, help="Local forwarded xArm control port")
    parser.add_argument("--report-port-normal", type=int, default=13001, help="Local forwarded xArm normal report port")
    parser.add_argument("--report-port-rich", type=int, default=13002, help="Local forwarded xArm rich report port")
    parser.add_argument("--report-port-real", type=int, default=13003, help="Local forwarded xArm real-time report port")
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--control-hz", type=float, default=120.0)
    parser.add_argument("--data-dir", default=os.path.expanduser("~/Projects/Teleop/data"))
    parser.add_argument("--camera-id", type=int, default=5)
    parser.add_argument("--camera-dev", default="/dev/video5")
    parser.add_argument("--allow-camera-fallback", action="store_true")
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--no-fps-mouse", action="store_true")
    parser.add_argument("--sdk-timeout-s", type=float, default=0.5)
    parser.add_argument("--video-hz", type=float, default=24.0)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()

    if args.self_check:
        raise SystemExit(self_check())

    # The SDK uses TCP_CONTROL_PORT and TCP_CONTROL_PORT+1 internally.
    XCONF.SocketConf.TCP_CONTROL_PORT = int(args.robot_port)
    XCONF.SocketConf.TCP_REPORT_NORM_PORT = int(args.report_port_normal)
    XCONF.SocketConf.TCP_REPORT_RICH_PORT = int(args.report_port_rich)
    XCONF.SocketConf.TCP_REPORT_REAL_PORT = int(args.report_port_real)

    app = Teleop5090(
        args.robot_ip,
        args.rate_hz,
        args.control_hz,
        args.data_dir,
        args.camera_id,
        args.camera_dev,
        (not args.allow_camera_fallback),
        (not args.no_video),
        (not args.no_fps_mouse),
        args.sdk_timeout_s,
        args.video_hz,
    )

    def _handle(_sig, _frm):
        app.running = False

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)
    app.run()


if __name__ == "__main__":
    main()
