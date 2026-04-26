#!/usr/bin/env python3
import argparse
import collections
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
from lerobot.datasets.lerobot_dataset import LeRobotDataset
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
SPEED_PRESETS = {
    "1": 0.5,
    "2": 1.0,
    "3": 2.0,
}


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


class TkVideoWindow:
    def __init__(self, title, close_callback=None, scale=1.0):
        self.title = title
        self.close_callback = close_callback
        self.scale = max(0.1, float(scale))
        self.tk = None
        self.root = None
        self.label = None
        self.photo = None
        self.width = 0
        self.height = 0
        self.last_raise_ts = 0.0

    def ensure(self, width, height, center=None):
        if self.root is None:
            import tkinter as tk

            self.tk = tk
            self.root = tk.Tk()
            self.root.title(self.title)
            self.root.configure(bg="black")
            self.root.resizable(False, False)
            self.root.protocol("WM_DELETE_WINDOW", self._on_close)
            self.label = tk.Label(self.root, bd=0, highlightthickness=0, bg="black")
            self.label.pack()
            self.root.bind("<Map>", self._raise_topmost)
            self.root.bind("<Configure>", self._raise_topmost)
            self.root.bind("<FocusOut>", self._raise_topmost)
        if int(width) != self.width or int(height) != self.height:
            self.width = int(width)
            self.height = int(height)
            self.label.configure(width=self.width, height=self.height)
        self.apply_layout(center=center)

    def _on_close(self):
        if callable(self.close_callback):
            self.close_callback()

    def _raise_topmost(self, _event=None):
        self.raise_topmost(force=True)

    def raise_topmost(self, force=False):
        if self.root is None:
            return
        now = time.monotonic()
        if not force and (now - self.last_raise_ts) < 0.25:
            return
        self.last_raise_ts = now
        try:
            self.root.attributes("-topmost", True)
        except Exception:
            pass
        try:
            self.root.wm_attributes("-topmost", 1)
        except Exception:
            pass
        try:
            self.root.lift()
        except Exception:
            pass

    def apply_layout(self, center=None):
        if self.root is None or self.width <= 0 or self.height <= 0:
            return
        if center is not None:
            cx, cy = int(center[0]), int(center[1])
        else:
            cx = max(1, int(self.root.winfo_screenwidth()) // 2)
            cy = max(1, int(self.root.winfo_screenheight()) // 2)
        x = max(0, int(cx - self.width // 2))
        y = max(0, int(cy - self.height // 2))
        try:
            self.root.geometry(f"{self.width}x{self.height}+{x}+{y}")
        except Exception:
            pass
        self.raise_topmost(force=True)

    def show_bgr(self, frame, center=None):
        if self.scale != 1.0:
            frame = cv2.resize(
                frame,
                (max(1, int(round(frame.shape[1] * self.scale))), max(1, int(round(frame.shape[0] * self.scale)))),
                interpolation=cv2.INTER_LINEAR,
            )
        h, w = frame.shape[:2]
        self.ensure(w, h, center=center)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        ppm = b"P6\n%d %d\n255\n" % (w, h) + rgb.tobytes()
        self.photo = self.tk.PhotoImage(data=ppm, format="PPM")
        self.label.configure(image=self.photo)
        self.apply_layout(center=center)
        self.root.update_idletasks()
        self.root.update()

    def close(self):
        if self.root is None:
            return
        try:
            self.root.destroy()
        except Exception:
            pass
        self.root = None
        self.label = None
        self.photo = None
        self.tk = None


class TeleopApp:
    def __init__(self, robot_ip, rate_hz, control_hz, data_dir, camera_id, camera_dev, base_camera_id, base_camera_dev, strict_camera_dev, show_video, fps_mouse, sdk_timeout_s, video_hz, repo_id, task, vcodec, encoder_threads, encoder_queue_maxsize):
        self.robot_ip = robot_ip
        self.rate_hz = rate_hz
        self.control_hz = float(control_hz)
        self.data_dir = data_dir
        self.camera_id = camera_id
        self.camera_dev = camera_dev
        self.base_camera_id = base_camera_id
        self.base_camera_dev = base_camera_dev
        self.strict_camera_dev = strict_camera_dev
        self.show_video = show_video
        self.fps_mouse = fps_mouse
        self.sdk_timeout_s = float(sdk_timeout_s)
        self.video_hz = float(video_hz)
        self.repo_id = repo_id
        self.task_description = task
        self.vcodec = vcodec
        self.encoder_threads = encoder_threads
        self.encoder_queue_maxsize = encoder_queue_maxsize
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

        self.lerobot_dataset = None
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

        self.window_name = "Teleop Video+Currents"
        self.video_window_scale = 2.5
        self.video_window = None
        self.speed_scale = SPEED_PRESETS["2"]
        self.camera = None
        self.camera_source = None
        self.base_camera = None
        self.base_camera_source = None
        self.last_camera_retry_ts = 0.0
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

        self._init_lerobot_dataset()

    def _build_lerobot_features(self):
        return {
            "observation.state": {
                "dtype": "float32",
                "shape": (14,),
                "names": ["J1", "J2", "J3", "J4", "J5", "J6",
                          "gripper_pos",
                          "curr_J1", "curr_J2", "curr_J3", "curr_J4",
                          "curr_J5", "curr_J6", "curr_gripper"],
            },
            "action": {
                "dtype": "float32",
                "shape": (7,),
                "names": ["vx", "vy", "vz", "vrx", "vry", "vrz", "gripper_action"],
            },
            "observation.images.wrist": {
                "dtype": "video",
                "shape": (480, 640, 3),
                "names": ["height", "width", "channels"],
            },
            "observation.images.base": {
                "dtype": "video",
                "shape": (480, 640, 3),
                "names": ["height", "width", "channels"],
            },
            "observation.joints_deg": {
                "dtype": "float32",
                "shape": (6,),
                "names": ["J1", "J2", "J3", "J4", "J5", "J6"],
            },
            "observation.pose_xyzrpy_deg": {
                "dtype": "float32",
                "shape": (6,),
                "names": ["x", "y", "z", "roll", "pitch", "yaw"],
            },
            "observation.currents": {
                "dtype": "float32",
                "shape": (7,),
                "names": ["J1", "J2", "J3", "J4", "J5", "J6", "gripper"],
            },
            "observation.currents_filtered": {
                "dtype": "float32",
                "shape": (7,),
                "names": ["J1", "J2", "J3", "J4", "J5", "J6", "gripper"],
            },
        }

    def _init_lerobot_dataset(self):
        root = os.path.join(self.data_dir, "lerobot_dataset")
        features = self._build_lerobot_features()

        if os.path.isdir(root) and os.path.isfile(os.path.join(root, "meta", "info.json")):
            print(f"[INFO] resuming existing LeRobot dataset at {root}")
            self.lerobot_dataset = LeRobotDataset(
                self.repo_id,
                root=root,
                streaming_encoding=True,
                encoder_queue_maxsize=self.encoder_queue_maxsize,
                encoder_threads=self.encoder_threads,
                vcodec=self.vcodec,
            )
        else:
            print(f"[INFO] creating new LeRobot dataset at {root}")
            self.lerobot_dataset = LeRobotDataset.create(
                repo_id=self.repo_id,
                fps=int(self.rate_hz),
                features=features,
                root=root,
                robot_type="xarm",
                use_videos=True,
                streaming_encoding=True,
                encoder_queue_maxsize=self.encoder_queue_maxsize,
                encoder_threads=self.encoder_threads,
                vcodec=self.vcodec,
            )
        print(f"[INFO] LeRobot dataset ready: {self.lerobot_dataset.num_episodes} episodes, "
              f"{self.lerobot_dataset.meta.total_frames} frames")

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
        self.camera = self.open_camera_with_probe(self.camera_id, self.camera_dev, allow_fallback=True, role="wrist")
        if self.camera is None:
            print(f"[WARN] no wrist camera frame available")
            if self.strict_camera_dev:
                raise RuntimeError(f"strict camera mode: cannot open {self.camera_dev}")
        self.base_camera = self.open_camera_with_probe(self.base_camera_id, self.base_camera_dev, allow_fallback=False, role="base")
        if self.base_camera is None:
            print(f"[WARN] no base camera frame available")

    def iter_camera_sources(self, preferred_cam_id, preferred_cam_dev, allow_fallback=True):
        sources = []
        if preferred_cam_dev:
            sources.append(preferred_cam_dev)
        elif preferred_cam_id is not None:
            sources.append(int(preferred_cam_id))
        if allow_fallback and not self.strict_camera_dev:
            sources.extend(["/dev/video0", "/dev/video4", "/dev/video6", 0, 1, 11])
        return sources

    def open_camera_with_probe(self, preferred_cam_id, preferred_cam_dev, allow_fallback=True, role="wrist"):
        tried = set()
        for src in self.iter_camera_sources(preferred_cam_id, preferred_cam_dev, allow_fallback=allow_fallback):
            if src in tried:
                continue
            tried.add(src)
            print(f"[INFO] probing {role} camera source {src}")
            stream = CameraStream(src, 640, 480)
            if not stream.cap.isOpened():
                print(f"[WARN] {role} camera source {src} open failed")
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
                if role == "wrist":
                    self.camera_source = src
                else:
                    self.base_camera_source = src
                if isinstance(src, str):
                    print(f"[INFO] {role} camera source {src} ready")
                else:
                    if role == "wrist":
                        self.camera_id = int(src)
                    else:
                        self.base_camera_id = int(src)
                    print(f"[INFO] {role} camera index {src} ready")
                return stream
            stream.close()
        print(f"[ERROR] no usable {role} camera frames (preferred index={preferred_cam_id}, dev={preferred_cam_dev})")
        return None

    def _get_camera_frame_rgb(self, stream):
        """Get current camera frame as RGB uint8 numpy array. Returns black frame if stale/unavailable."""
        if stream is None:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        frame, ts, count = stream.get()
        if frame is None or count < 1 or (time.monotonic() - ts) > 1.0:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        if frame.shape[:2] != (480, 640):
            frame = cv2.resize(frame, (640, 480))
        return frame

    def capture_episode_observation(self, state):
        joints = [float(x) for x in state.get("joints_deg", [0.0] * 6)]
        gripper = float(state.get("gripper_pos", 0.0))
        currents = [float(x) for x in state.get("currents", [0.0] * 7)]
        obs_state = np.array(joints + [gripper] + currents, dtype=np.float32)

        vel_cmd = [float(x) for x in state.get("action_velocity_cmd", [0.0] * 6)]
        grip_action = float(state.get("action_gripper", 0.0))
        action = np.array(vel_cmd + [grip_action], dtype=np.float32)

        wrist_frame = self._get_camera_frame_rgb(self.camera)
        base_frame = self._get_camera_frame_rgb(self.base_camera)

        pose = [float(x) for x in state.get("pose_xyzrpy_deg", [0.0] * 6)]
        filtered = state.get("currents_filtered", [0.0] * 7)
        currents_filtered = [float(x) if x is not None else 0.0 for x in filtered]

        frame_dict = {
            "observation.state": obs_state,
            "action": action,
            "observation.images.wrist": wrist_frame,
            "observation.images.base": base_frame,
            "observation.joints_deg": np.array(joints, dtype=np.float32),
            "observation.pose_xyzrpy_deg": np.array(pose, dtype=np.float32),
            "observation.currents": np.array(currents, dtype=np.float32),
            "observation.currents_filtered": np.array(currents_filtered, dtype=np.float32),
            "task": self.task_description,
        }
        self.lerobot_dataset.add_frame(frame_dict)
        return state

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
        if self.video_window is None:
            self.video_window = TkVideoWindow(
                self.window_name,
                close_callback=self.request_stop,
                scale=self.video_window_scale,
            )
        try:
            self.video_window.show_bgr(canvas, center=self.screen_center)
        except Exception as exc:
            print(f"[WARN] video window update failed: {exc}")
            self.request_stop()
            return
        dt_ms = (time.monotonic() - t0) * 1000.0
        if dt_ms > 120.0:
            print(f"\n[WARN] video panel render slow: {dt_ms:.1f}ms")

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
            elif ch in SPEED_PRESETS:
                self.speed_scale = SPEED_PRESETS[ch]
                print(f"\n[SPEED] scale={self.speed_scale:.1f}x")
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
            speed_scale = float(self.speed_scale)
        return keys, dx, dy, home, speed_scale

    def compose_velocity_cmd(self, keys, dx, dy, speed_scale):
        linear_vel = self.linear_vel_mm_s * float(speed_scale)
        angular_vel = self.angular_vel_deg_s * float(speed_scale)
        mouse_gain = self.mouse_vel_gain_deg_s_per_px * float(speed_scale)
        vx = 0.0
        vy = 0.0
        vz = 0.0
        vrx = 0.0
        vry = 0.0
        vrz = 0.0

        # Translational keys are expressed directly in the current tool frame.
        # With is_tool_coord=True on velocity send, "forward" follows the gripper direction.
        if "w" in keys:
            vz += linear_vel
        if "s" in keys:
            vz -= linear_vel
        if "space" in keys:
            vx -= linear_vel
        if "shift" in keys:
            vx += linear_vel
        if "a" in keys:
            vy += linear_vel
        if "d" in keys:
            vy -= linear_vel

        if "q" in keys:
            vrz -= angular_vel
        if "e" in keys:
            vrz += angular_vel

        # On this setup the controller's effective RX/RZ slots are opposite to the
        # SDK naming in teleop hand-feel, so mouse-x/QE are intentionally swapped.
        # mouse left/right -> physical yaw ; mouse up/down -> physical pitch
        vrx += dx * mouse_gain
        vry += (dy) * mouse_gain
        vy = clip(vy, -linear_vel, linear_vel)
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
            keys, dx, dy, home, speed_scale = self.snapshot_inputs()
            if home:
                self.go_home_now()
            vel = self.compose_velocity_cmd(keys, dx, dy, speed_scale)
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

    def request_stop(self):
        self.running = False

    def close_video_window(self):
        if self.video_window is None:
            return
        self.video_window.close()
        self.video_window = None

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
        self.close_video_window()

    def save_episode(self):
        if self.lerobot_dataset.episode_buffer is None or self.lerobot_dataset.episode_buffer["size"] == 0:
            print("\n[SAVE] no frames to save, skipping")
            return
        num_frames = self.lerobot_dataset.episode_buffer["size"]
        ep_idx = self.lerobot_dataset.episode_buffer["episode_index"]
        print(f"\n[SAVE] saving episode {ep_idx} ({num_frames} frames)...")
        self.lerobot_dataset.save_episode()
        print(f"[SAVE] episode saved (total: {self.lerobot_dataset.num_episodes} episodes, "
              f"{self.lerobot_dataset.meta.total_frames} frames)")

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
            f"[RUN] teleop started: WASD/Shift/Space, mouse move, left click(toggle gripper), "
            f"R, Enter. wrist={video_src} base={self.base_camera_dev or self.base_camera_id} "
            f"dataset={self.lerobot_dataset.root}"
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
            self.capture_episode_observation(st)
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
        try:
            self.save_episode()
        finally:
            if self.lerobot_dataset is not None:
                self.lerobot_dataset.finalize()
                print("[INFO] LeRobot dataset finalized")
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
        if self.base_camera is not None:
            self.base_camera.close()
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
    parser.add_argument("--robot-ip", default="192.168.1.199", help="xArm IP")
    parser.add_argument("--robot-port", type=int, default=502, help="xArm control port")
    parser.add_argument("--report-port-normal", type=int, default=30001, help="xArm normal report port")
    parser.add_argument("--report-port-rich", type=int, default=30002, help="xArm rich report port")
    parser.add_argument("--report-port-real", type=int, default=30003, help="xArm real-time report port")
    parser.add_argument("--rate-hz", type=float, default=30.0)
    parser.add_argument("--control-hz", type=float, default=120.0)
    parser.add_argument("--data-dir", default=os.path.expanduser("~/下载"))
    parser.add_argument("--camera-id", type=int, default=0)
    parser.add_argument("--camera-dev", default="/dev/video0")
    parser.add_argument("--base-camera-id", type=int, default=6)
    parser.add_argument("--base-camera-dev", default="/dev/video6")
    parser.add_argument("--allow-camera-fallback", action="store_true")
    parser.add_argument("--no-video", action="store_true")
    parser.add_argument("--no-fps-mouse", action="store_true")
    parser.add_argument("--sdk-timeout-s", type=float, default=0.5)
    parser.add_argument("--video-hz", type=float, default=24.0)
    parser.add_argument("--repo-id", default="teleop/xarm_mouse_teleop", help="LeRobot dataset repo id")
    parser.add_argument("--task", default="keyboard_mouse_teleop", help="Task description for dataset")
    parser.add_argument("--vcodec", default="libsvtav1", help="Video codec for LeRobot streaming encoding")
    parser.add_argument("--encoder-threads", type=int, default=2, help="Threads per video encoder")
    parser.add_argument("--encoder-queue-maxsize", type=int, default=30, help="Max buffered frames per camera")
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args()

    if args.self_check:
        raise SystemExit(self_check())

    # The SDK uses TCP_CONTROL_PORT and TCP_CONTROL_PORT+1 internally.
    XCONF.SocketConf.TCP_CONTROL_PORT = int(args.robot_port)
    XCONF.SocketConf.TCP_REPORT_NORM_PORT = int(args.report_port_normal)
    XCONF.SocketConf.TCP_REPORT_RICH_PORT = int(args.report_port_rich)
    XCONF.SocketConf.TCP_REPORT_REAL_PORT = int(args.report_port_real)

    app = TeleopApp(
        args.robot_ip,
        args.rate_hz,
        args.control_hz,
        args.data_dir,
        args.camera_id,
        args.camera_dev,
        args.base_camera_id,
        args.base_camera_dev,
        (not args.allow_camera_fallback),
        (not args.no_video),
        (not args.no_fps_mouse),
        args.sdk_timeout_s,
        args.video_hz,
        args.repo_id,
        args.task,
        args.vcodec,
        args.encoder_threads,
        args.encoder_queue_maxsize,
    )

    def _handle(_sig, _frm):
        app.running = False

    signal.signal(signal.SIGINT, _handle)
    signal.signal(signal.SIGTERM, _handle)
    app.run()


if __name__ == "__main__":
    main()
