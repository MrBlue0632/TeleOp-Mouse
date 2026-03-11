#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import os
import shlex
import struct
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np

JOINT_LIMITS_RAD = {
    0: (np.deg2rad(-360), np.deg2rad(360)),
    1: (np.deg2rad(-117), np.deg2rad(116)),
    2: (np.deg2rad(-219), np.deg2rad(10)),
    3: (np.deg2rad(100), np.deg2rad(283)),
    4: (np.deg2rad(-20), np.deg2rad(180)),
    5: (np.deg2rad(-100), np.deg2rad(80)),
}

# Fixed panel max for J1~J6 (abs current), tuned from recorded log maxima * 1.5.
FIXED_JOINT_BAR_MAXIMA = np.array([2.7712, 9.6113, 5.5237, 1.7850, 1.4963, 1.2788], dtype=np.float64)


@dataclass
class LatestFrame:
    frame: Any | None = None
    ts: float = 0.0
    count: int = 0


class CameraStream:
    def __init__(self, cv2_mod, cam_id: int, width: int = 640, height: int = 480):
        self.cv2 = cv2_mod
        self.cap = cv2_mod.VideoCapture(cam_id, cv2_mod.CAP_V4L2)
        if self.cap.isOpened():
            self.cap.set(cv2_mod.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2_mod.CAP_PROP_FRAME_HEIGHT, height)
            # Keep latency low for realtime display.
            try:
                self.cap.set(cv2_mod.CAP_PROP_BUFFERSIZE, 1)
            except Exception:
                pass
        self._latest = LatestFrame()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._loop, daemon=True)
        self._started = False

    def start(self) -> None:
        if not self._started:
            self._th.start()
            self._started = True

    def stop(self) -> None:
        self._stop.set()
        if self._started:
            self._th.join(timeout=1.0)
        self.cap.release()

    def _loop(self) -> None:
        while not self._stop.is_set():
            try:
                if not self.cap.isOpened():
                    time.sleep(0.05)
                    continue
                ok, frame = self.cap.read()
                if ok and frame is not None:
                    with self._lock:
                        self._latest.frame = frame
                        self._latest.ts = time.monotonic()
                        self._latest.count += 1
                else:
                    time.sleep(0.005)
            except Exception:
                time.sleep(0.01)

    def get(self):
        with self._lock:
            return self._latest.frame, self._latest.ts, self._latest.count


def _import_cv2():
    try:
        import cv2  # type: ignore
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError("Missing OpenCV. Install with: pip install opencv-python") from exc
    return cv2


def _import_xarm():
    try:
        from xarm.wrapper import XArmAPI  # type: ignore
    except Exception as exc:  # noqa: BLE001
        raise RuntimeError("Missing xArm SDK. Install with: pip install xarm-python-sdk") from exc
    return XArmAPI


def load_remote_episode(remote_user: str, remote_host: str, remote_dataset_dir: str, episode_id: int) -> np.ndarray:
    remote_file = f"{remote_dataset_dir.rstrip('/')}/episode_{episode_id}_state.json"
    remote_cmd = f"cat {shlex.quote(remote_file)}"
    proc = subprocess.run(["ssh", f"{remote_user}@{remote_host}", remote_cmd], capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise RuntimeError(f"Failed to fetch {remote_file}: {proc.stderr.strip()}")

    raw: Any = json.loads(proc.stdout)
    if not isinstance(raw, list) or not raw:
        raise RuntimeError(f"Unexpected episode format in {remote_file}")
    arr = np.array(raw, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] < 7:
        raise RuntimeError(f"Episode shape invalid: {arr.shape}, expected [N, >=7]")
    return arr


def load_local_jsonl_episode(jsonl_file: str) -> np.ndarray:
    rows: list[list[float]] = []
    last_gripper = 0.0
    with open(jsonl_file, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            joints = d.get("joints_deg")
            if not isinstance(joints, list) or len(joints) < 6:
                continue
            g = d.get("gripper_pos", last_gripper)
            try:
                g = float(g)
            except Exception:
                g = last_gripper
            last_gripper = g
            rows.append([float(joints[i]) for i in range(6)] + [float(g)])
    if not rows:
        raise RuntimeError(f"No valid rows in jsonl: {jsonl_file}")
    return np.array(rows, dtype=np.float64)


def init_arm(arm_ip: str, retries: int = 4, retry_sleep: float = 1.0):
    XArmAPI = _import_xarm()
    last_exc = None
    for i in range(retries):
        try:
            print(f"[INFO] connecting arm via {arm_ip}:502 ... (attempt {i+1}/{retries})")
            arm = XArmAPI(arm_ip)
            arm.set_gripper_enable(enable=True)
            time.sleep(0.3)
            return arm
        except Exception as exc:  # noqa: BLE001
            last_exc = exc
            time.sleep(retry_sleep)
    raise RuntimeError(f"connect arm failed after {retries} attempts: {last_exc}")


def clamp_rad(angles: np.ndarray) -> np.ndarray:
    out = angles.copy()
    for i, (lo, hi) in JOINT_LIMITS_RAD.items():
        out[i] = np.clip(out[i], lo, hi)
    return out


def unwrap_clamp(curr: np.ndarray, target: np.ndarray) -> np.ndarray:
    delta = target - curr
    delta = (delta + np.pi) % (2 * np.pi) - np.pi
    return clamp_rad(curr + delta)


def prepare_arm_for_replay(arm, retries: int = 3) -> bool:
    for i in range(retries):
        try:
            arm.clean_error()
            arm.set_state(0)
            arm.motion_enable(enable=True)
            arm.set_mode(6)
            arm.set_state(0)
            time.sleep(0.2)
            code, ang = arm.get_servo_angle(is_radian=True)
            if code == 0 and isinstance(ang, list):
                # Preflight: a no-op servo command must succeed before replay.
                ret = arm.set_servo_angle(angle=list(np.array(ang[:6], dtype=float)), speed=float(np.deg2rad(20.0)), is_radian=True)
                if ret == 0:
                    print(f"[INFO] arm preflight ok (attempt {i+1})")
                    return True
        except Exception:
            pass
        time.sleep(0.3)
    return False


def goto_init(arm, state0: np.ndarray) -> None:
    init_deg = [float(x) for x in state0[:6]]
    grip = float(state0[6])
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.2)
    ret = arm.set_servo_angle(angle=init_deg, speed=20, is_radian=False, wait=False)
    print(f"[INFO] goto_init ret={ret}")
    arm.set_gripper_position(pos=grip, wait=False)
    time.sleep(1.0)
    arm.set_mode(6)
    arm.set_state(0)
    time.sleep(0.4)


def recover_arm_ready(arm) -> None:
    try:
        arm.clean_error()
    except Exception:
        pass
    try:
        arm.set_mode(0)
        arm.set_state(0)
        time.sleep(0.05)
        arm.set_state(0)
        arm.motion_enable(enable=True)
        arm.set_mode(6)
        arm.set_state(0)
        time.sleep(0.05)
    except Exception:
        pass


def send_servo_with_retry(arm, target_rad: np.ndarray, speed_rad_s: float, retries: int = 3) -> int:
    for k in range(retries):
        ret = arm.set_servo_angle(angle=list(target_rad), speed=float(speed_rad_s), is_radian=True)
        if ret == 0:
            return 0
        recover_arm_ready(arm)
        time.sleep(0.02 * (k + 1))
    return ret


def draw_current_panel(
    cv2_mod,
    panel,
    filtered_curr: np.ndarray,
    joint_current_maxima: np.ndarray,
    gripper_current_max: float,
) -> None:
    panel[:] = (22, 24, 28)
    labels = ["J1", "J2", "J3", "J4", "J5", "J6", "GRIP"]
    n = len(labels)
    h, w = panel.shape[:2]
    top = 32
    row_h = (h - 48) // n
    bar_x = 200
    bar_w = w - bar_x - 16
    title = "Currents"
    cv2_mod.putText(panel, title, (12, 22), cv2_mod.FONT_HERSHEY_SIMPLEX, 0.62, (210, 210, 220), 1)
    for i, lab in enumerate(labels):
        y = top + i * row_h
        val = float(abs(filtered_curr[i])) if not np.isnan(filtered_curr[i]) else float("nan")
        cv2_mod.putText(panel, lab, (14, y + 20), cv2_mod.FONT_HERSHEY_SIMPLEX, 0.66, (220, 220, 220), 1)
        txt = "N/A" if np.isnan(val) else f"{val:.2f}"
        cv2_mod.putText(panel, txt, (145, y + 20), cv2_mod.FONT_HERSHEY_SIMPLEX, 0.66, (235, 235, 235), 1)
        vmax = float(gripper_current_max if i == 6 else joint_current_maxima[i])
        vmax = max(1e-6, vmax)
        ratio = 0.0 if np.isnan(val) else min(1.0, val / vmax)
        fill = int(bar_w * ratio)
        cv2_mod.rectangle(panel, (bar_x, y + 4), (bar_x + bar_w, y + 24), (70, 70, 78), -1)
        color = (50, 190, 255) if ratio < 0.75 else (60, 120, 255)
        cv2_mod.rectangle(panel, (bar_x, y + 4), (bar_x + fill, y + 24), color, -1)
        cv2_mod.rectangle(panel, (bar_x, y + 4), (bar_x + bar_w, y + 24), (110, 110, 120), 1)


def draw_3d_arrow(cv2_mod, frame, delta_deg: np.ndarray) -> None:
    h, w = frame.shape[:2]
    cx, cy = w // 2, h // 2

    # Map current action direction/size from first 3 joints.
    vx = float(delta_deg[0])
    vy = float(-delta_deg[1])
    vz = float(delta_deg[2])
    mag = float(np.linalg.norm([vx, vy, vz]))

    if mag < 1e-4:
        return

    scale = min(120.0, 8.0 * mag)
    ex = int(cx + np.clip(vx, -8, 8) / 8.0 * scale)
    ey = int(cy + np.clip(vy, -8, 8) / 8.0 * scale)

    # pseudo depth: z controls shadow offset and thickness
    z_norm = np.clip(vz / 8.0, -1.0, 1.0)
    depth = int(8 * z_norm)
    thickness = 3 + int(3 * abs(z_norm))

    # shadow layer
    cv2_mod.arrowedLine(frame, (cx + depth, cy + depth), (ex + depth, ey + depth), (18, 18, 18), thickness + 3, tipLength=0.22)
    # core layer
    cv2_mod.arrowedLine(frame, (cx, cy), (ex, ey), (40, 230, 255), thickness, tipLength=0.22)
    # highlight edge
    cv2_mod.arrowedLine(frame, (cx - 1, cy - 1), (ex - 1, ey - 1), (220, 255, 255), 1, tipLength=0.22)


def draw_3d_eef_vector(
    cv2_mod,
    frame,
    vec_xyz_mm: np.ndarray,
    yaw_offset_deg: float = 0.0,
) -> None:
    """Draw a simple 3D axis at center and the EEF lookahead vector in that coordinate."""
    h, w = frame.shape[:2]
    origin = np.array([w * 0.5, h * 0.5], dtype=np.float64)

    # Weak perspective projection from 3D(x,y,z) to 2D(u,v):
    # Visual convention for this setup:
    # - +Y (green) points to image left
    # - +X (red) is shown as "into screen" depth (diagonal down-right)
    # - +Z keeps up/down cue
    def proj(v3: np.ndarray) -> np.ndarray:
        x, y, z = float(v3[0]), float(v3[1]), float(v3[2])
        # Flip x depth sign so the yellow vector front/back matches real motion.
        u = -0.95 * y - 0.28 * x
        v = -1.00 * z - 0.22 * x
        return np.array([u, v], dtype=np.float64)

    # vec_xyz_mm from FK is already in robot base frame.
    # Only apply a fixed camera-vs-base yaw calibration, avoid per-frame base rotation.
    yaw = np.deg2rad(yaw_offset_deg)
    c, s = np.cos(yaw), np.sin(yaw)
    rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)

    axis_len_mm = 35.0
    px_per_mm = 0.55
    axes = [
        (np.array([axis_len_mm, 0.0, 0.0]), (40, 90, 255)),   # X
        (np.array([0.0, axis_len_mm, 0.0]), (70, 220, 70)),   # Y
        (np.array([0.0, 0.0, axis_len_mm]), (255, 170, 60)),  # Z
    ]

    # Draw center halo for depth feel.
    cv2_mod.circle(frame, (int(origin[0]), int(origin[1])), 7, (20, 20, 20), -1)
    cv2_mod.circle(frame, (int(origin[0]), int(origin[1])), 4, (220, 220, 220), -1)

    for axis_v, color in axes:
        axis_v_rot = rz @ axis_v
        p2 = origin + proj(axis_v_rot) * px_per_mm
        cv2_mod.arrowedLine(
            frame,
            (int(origin[0]), int(origin[1])),
            (int(p2[0]), int(p2[1])),
            color,
            2,
            tipLength=0.15,
        )

    v = vec_xyz_mm.astype(np.float64) * 10.0
    v = rz @ v
    mag = float(np.linalg.norm(v))
    if mag < 1e-6:
        return

    v_cap = v * min(1.0, 260.0 / mag)
    p_shadow = origin + proj(v_cap) * px_per_mm + np.array([5.0, 5.0])
    p_vec = origin + proj(v_cap) * px_per_mm

    cv2_mod.arrowedLine(
        frame,
        (int(origin[0] + 5), int(origin[1] + 5)),
        (int(p_shadow[0]), int(p_shadow[1])),
        (18, 18, 18),
        6,
        tipLength=0.18,
    )
    cv2_mod.arrowedLine(
        frame,
        (int(origin[0]), int(origin[1])),
        (int(p_vec[0]), int(p_vec[1])),
        (35, 245, 245),
        4,
        tipLength=0.18,
    )


def wait_camera_ready(stream: CameraStream, timeout_sec: float = 8.0) -> None:
    stream.start()
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_sec:
        frame, ts, count = stream.get()
        if frame is not None and count >= 1 and (time.monotonic() - ts) < 1.0:
            return
        time.sleep(0.03)
    raise RuntimeError("Camera not ready (no frame in time)")


def read_currents(arm) -> np.ndarray:
    vals = np.full((7,), np.nan, dtype=np.float64)
    try:
        cur = list(arm.currents)
        for i in range(min(6, len(cur))):
            vals[i] = float(cur[i])
    except Exception:
        pass
    return vals


def read_gripper_current_register(arm, register_addr: int = 0x0003) -> float:
    """
    Read gripper-related current from register path.
    Note: exact register map depends on gripper model/firmware.
    """
    # Align with 4060 monitor_gripper_current.py:
    # ret = arm.core.gripper_modbus_r16s(0x0003, 1), decode signed int16.
    try:
        if hasattr(arm, "core") and hasattr(arm.core, "gripper_modbus_r16s"):
            ret = arm.core.gripper_modbus_r16s(register_addr, 1)
            if isinstance(ret, (list, tuple)) and len(ret) >= 7 and ret[0] == 0:
                return float(struct.unpack(">h", bytes(ret[5:7]))[0])
    except Exception:
        pass
    return float("nan")


def precompute_eef_xyz_from_states(arm, states: np.ndarray) -> np.ndarray | None:
    xyz = np.zeros((len(states), 3), dtype=np.float64)
    ok_cnt = 0
    for i, s in enumerate(states):
        try:
            code, pose = arm.get_forward_kinematics(list(map(float, s[:6])), input_is_radian=False, return_is_radian=False)
            if code == 0 and isinstance(pose, (list, tuple)) and len(pose) >= 3:
                xyz[i] = np.array(pose[:3], dtype=np.float64)
                ok_cnt += 1
            elif i > 0:
                xyz[i] = xyz[i - 1]
        except Exception:
            if i > 0:
                xyz[i] = xyz[i - 1]
    if ok_cnt < max(10, len(states) // 5):
        return None
    return xyz


def load_eef_cache(cache_file: str, expect_steps: int) -> np.ndarray | None:
    try:
        arr = np.load(cache_file)
        if not isinstance(arr, np.ndarray):
            return None
        if arr.ndim != 2 or arr.shape != (expect_steps, 3):
            return None
        return arr.astype(np.float64)
    except Exception:
        return None


def save_eef_cache(cache_file: str, xyz: np.ndarray) -> None:
    try:
        os.makedirs(os.path.dirname(cache_file), exist_ok=True)
    except Exception:
        pass
    np.save(cache_file, xyz)


def resolve_latest_current_log(log_dir: str) -> str | None:
    if not os.path.isdir(log_dir):
        return None
    cands = []
    for name in os.listdir(log_dir):
        if not name.endswith(".csv"):
            continue
        p = os.path.join(log_dir, name)
        try:
            cands.append((os.path.getmtime(p), p))
        except Exception:
            continue
    if not cands:
        return None
    cands.sort(key=lambda x: x[0], reverse=True)
    return cands[0][1]


def load_joint_bar_max_from_log(log_file: str, factor: float = 1.5) -> np.ndarray | None:
    cols = ["j1", "j2", "j3", "j4", "j5", "j6"]
    max_abs = np.zeros((6,), dtype=np.float64)
    has_data = np.zeros((6,), dtype=bool)
    try:
        with open(log_file, "r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                for i, c in enumerate(cols):
                    s = str(row.get(c, "")).strip()
                    if not s:
                        continue
                    try:
                        v = abs(float(s))
                    except Exception:
                        continue
                    if v > max_abs[i]:
                        max_abs[i] = v
                    has_data[i] = True
    except Exception:
        return None
    if not bool(np.all(has_data)):
        return None
    return np.maximum(0.1, max_abs * float(factor))


def replay_realtime(
    cv2_mod,
    states: np.ndarray,
    stream: CameraStream | None,
    hz: float,
    arm=None,
    gripper_current_reg: int = 0x0003,
    send_gripper_cmd: bool = True,
    overlay_yaw_offset_deg: float = 0.0,
    gripper_current_max: float = 50.0,
    current_log_file: str | None = None,
    joint_bar_maxima: np.ndarray | None = None,
    eef_precompute: bool = False,
    eef_cache_file: str = "",
    show_hz_test: bool = True,
) -> None:
    dt = 1.0 / hz
    n_steps = len(states)
    panel_w = 360
    win_name = "Realtime"

    if arm is not None:
        _, raw = arm.get_servo_angle(is_radian=True)
        curr = np.array(raw[:6], dtype=np.float64)
    else:
        curr = np.deg2rad(states[0, :6]).astype(np.float64)

    filtered_curr = np.full((7,), np.nan, dtype=np.float64)
    curr_valid = np.zeros((7,), dtype=bool)
    run_joint_maxima = np.zeros((6,), dtype=np.float64)
    if joint_bar_maxima is None:
        joint_bar_maxima = np.full((6,), 1.0, dtype=np.float64)
    else:
        joint_bar_maxima = np.maximum(0.1, joint_bar_maxima.astype(np.float64))
    med_buf = [deque(maxlen=5) for _ in range(7)]
    alpha_rise = 0.35
    alpha_fall = 0.12
    prev_target_deg = np.array(states[0, :6], dtype=np.float64)
    last_gripper_cmd = None
    last_gripper_ts = 0.0
    cached_gripper_current = float("nan")
    last_gripper_current_ts = 0.0
    gripper_reg_enabled = True
    gripper_reg_fail_cnt = 0
    eef_xyz = None
    if arm is not None and eef_precompute:
        if eef_cache_file:
            eef_xyz = load_eef_cache(eef_cache_file, expect_steps=len(states))
            if eef_xyz is not None:
                print(f"[INFO] EEF cache hit: {eef_cache_file}")
        if eef_xyz is None:
            print("[INFO] precomputing EEF xyz from states ...")
            eef_xyz = precompute_eef_xyz_from_states(arm, states)
            if eef_xyz is None:
                print("[WARN] EEF precompute failed, vector overlay disabled")
            else:
                print("[INFO] EEF precompute done")
                if eef_cache_file:
                    save_eef_cache(eef_cache_file, eef_xyz)
                    print(f"[INFO] EEF cache saved: {eef_cache_file}")
        # Probe gripper register once. If failed, disable to avoid repeated RS485 stalls/spam.
        probe = read_gripper_current_register(arm, register_addr=gripper_current_reg)
        if np.isnan(probe):
            gripper_reg_enabled = False
            print(f"[WARN] gripper register 0x{gripper_current_reg:04X} unavailable, disable gripper current read")
        else:
            cached_gripper_current = probe
            last_gripper_current_ts = time.monotonic()

    cv2_mod.namedWindow(win_name, cv2_mod.WINDOW_NORMAL)
    next_t = time.monotonic()
    prev_step_t = None
    step_dt_hist: deque[float] = deque(maxlen=120)
    loop_cost_hist: deque[float] = deque(maxlen=120)
    overrun_count = 0
    log_fp = None
    if current_log_file:
        os.makedirs(os.path.dirname(current_log_file), exist_ok=True)
        log_fp = open(current_log_file, "w", encoding="utf-8")
        log_fp.write("step,ts,j1,j2,j3,j4,j5,j6,grip\n")

    try:
        for i, state in enumerate(states):
            now = time.monotonic()
            if now < next_t:
                time.sleep(next_t - now)
            step_t = time.monotonic()
            if prev_step_t is not None:
                step_dt_hist.append(step_t - prev_step_t)
            prev_step_t = step_t

            target_rad = unwrap_clamp(curr, np.deg2rad(state[:6]).astype(np.float64))
            target_deg = np.rad2deg(target_rad)
            gripper = float(state[6])

            if arm is not None:
                if getattr(arm, "error_code", 0) != 0:
                    recover_arm_ready(arm)
                # is_radian=True, so speed must be in rad/s.
                sret = send_servo_with_retry(arm, target_rad, speed_rad_s=float(np.deg2rad(40.0)), retries=3)
                if sret != 0 and (i % 30 == 0):
                    print(f"[WARN] servo command failed at step={i}, ret={sret}")
                now_t = time.monotonic()
                # Keep gripper trajectory active (same behavior as 4060 playback).
                if send_gripper_cmd and (
                    last_gripper_cmd is None
                    or abs(gripper - last_gripper_cmd) >= 2.0
                    or (now_t - last_gripper_ts) >= 0.06
                ):
                    gpos = float(min(850.0, max(0.0, gripper)))
                    gret = arm.set_gripper_position(pos=gpos, wait=False)
                    if gret == 0:
                        last_gripper_cmd = gpos
                        last_gripper_ts = now_t
            curr = target_rad

            if stream is not None:
                frame, _, _ = stream.get()
            else:
                frame = None
            if frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
            else:
                frame = cv2_mod.resize(frame, (640, 480))

            # Arrow from current EEF to lookahead (i+3) EEF in xyz.
            if eef_xyz is not None:
                j = min(i + 3, n_steps - 1)
                vec_xyz = eef_xyz[j] - eef_xyz[i]
                draw_3d_eef_vector(
                    cv2_mod,
                    frame,
                    vec_xyz,
                    yaw_offset_deg=overlay_yaw_offset_deg,
                )
            else:
                delta_deg = target_deg - prev_target_deg
                delta_deg = (delta_deg + 180.0) % 360.0 - 180.0
                draw_3d_arrow(cv2_mod, frame, delta_deg)
            prev_target_deg = target_deg

            # Current panel with low-pass filter
            if arm is not None:
                # Poll currents at 15Hz to reduce SDK call overhead in 30Hz control loop.
                if (i % 2) == 0:
                    raw_curr = read_currents(arm)
                else:
                    raw_curr = np.full((7,), np.nan, dtype=np.float64)
                now_t = time.monotonic()
                if gripper_reg_enabled and (now_t - last_gripper_current_ts >= 3.00):
                    gcur = read_gripper_current_register(arm, register_addr=gripper_current_reg)
                    last_gripper_current_ts = now_t
                    if np.isnan(gcur):
                        gripper_reg_fail_cnt += 1
                        if gripper_reg_fail_cnt >= 3:
                            gripper_reg_enabled = False
                            print("[WARN] gripper register current read disabled (repeated modbus failures)")
                    else:
                        gripper_reg_fail_cnt = 0
                        cached_gripper_current = gcur
                raw_curr[6] = cached_gripper_current
                for k in range(7):
                    v = raw_curr[k]
                    if np.isnan(v):
                        continue
                    med_buf[k].append(float(v))
                    v_med = float(np.median(med_buf[k]))
                    if not curr_valid[k]:
                        filtered_curr[k] = v_med
                        curr_valid[k] = True
                    else:
                        a = alpha_rise if abs(v_med) >= abs(filtered_curr[k]) else alpha_fall
                        filtered_curr[k] = (1.0 - a) * filtered_curr[k] + a * v_med

                abs_joint = np.abs(raw_curr[:6])
                run_joint_maxima = np.maximum(run_joint_maxima, np.nan_to_num(abs_joint, nan=0.0))
                if log_fp is not None:
                    vals = [raw_curr[k] for k in range(7)]
                    sval = ",".join("" if np.isnan(v) else f"{float(v):.6f}" for v in vals)
                    log_fp.write(f"{i},{step_t:.6f},{sval}\n")

            panel = np.zeros((480, panel_w, 3), dtype=np.uint8)
            draw_current_panel(
                cv2_mod,
                panel,
                filtered_curr,
                joint_current_maxima=joint_bar_maxima,
                gripper_current_max=gripper_current_max,
            )

            canvas = np.hstack([frame, panel])
            if show_hz_test:
                if step_dt_hist:
                    avg_dt = float(np.mean(np.array(step_dt_hist, dtype=np.float64)))
                    eff_hz = (1.0 / avg_dt) if avg_dt > 1e-6 else 0.0
                else:
                    eff_hz = 0.0
                loop_cost_ms = float((time.monotonic() - step_t) * 1000.0)
                loop_cost_hist.append(loop_cost_ms)
                mean_cost_ms = float(np.mean(np.array(loop_cost_hist, dtype=np.float64))) if loop_cost_hist else 0.0
                color = (40, 220, 120) if abs(eff_hz - hz) <= 1.0 else (0, 140, 255)
                cv2_mod.putText(
                    canvas,
                    f"HZ TEST  target={hz:.1f}  actual={eff_hz:.2f}  loop={mean_cost_ms:.1f}ms  overrun={overrun_count}",
                    (12, 24),
                    cv2_mod.FONT_HERSHEY_SIMPLEX,
                    0.58,
                    color,
                    2,
                )
            cv2_mod.imshow(win_name, canvas)
            key = cv2_mod.waitKey(1) & 0xFF
            if key == 27:
                break

            next_t += dt
            lag = time.monotonic() - next_t
            if lag > 0.25:
                # If GUI/system stalls, resync to avoid accumulating delay.
                overrun_count += 1
                next_t = time.monotonic() + dt
    finally:
        if log_fp is not None:
            log_fp.close()
            print(f"[INFO] current log saved: {current_log_file}")
        if step_dt_hist:
            avg_dt = float(np.mean(np.array(step_dt_hist, dtype=np.float64)))
            eff_hz = (1.0 / avg_dt) if avg_dt > 1e-6 else 0.0
            print(f"[INFO] hz test summary: target={hz:.2f}, actual={eff_hz:.3f}, overruns={overrun_count}")
        print(
            "[INFO] run joint max abs current: "
            + ", ".join(f"J{k+1}={run_joint_maxima[k]:.3f}" for k in range(6))
            + f", GRIP_MAX_FIXED={gripper_current_max:.3f}"
        )

    cv2_mod.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Replay 4060 trajectory + 5090 realtime camera")
    p.add_argument("episode_id", type=int, nargs="?", default=None, help="episode id under ~/ros_ws_zyl/dataset on 4060")
    p.add_argument("--jsonl-file", default="", help="local jsonl episode file (if set, ignore remote episode id)")
    p.add_argument("--remote-user", default="bozhao_4060_2")
    p.add_argument("--remote-host", default="192.168.31.249")
    p.add_argument("--remote-dataset-dir", default="/home/bozhao_4060_2/ros_ws_zyl/dataset")
    p.add_argument("--hz", type=float, default=30.0)
    p.add_argument("--camera-ids", default="4", help="local camera ids, first one is used")
    p.add_argument("--no-camera", action="store_true", help="skip camera init/display uses black canvas")
    p.add_argument("--arm-ip", default="127.0.0.1", help="use tunnel endpoint, default 127.0.0.1")
    p.add_argument("--no-arm", action="store_true", help="skip arm playback, camera+overlay only")
    p.add_argument("--gripper-current-reg", type=lambda x: int(x, 0), default=0x0003, help="gripper current register (monitor script uses 0x0003)")
    p.add_argument("--no-gripper-cmd", action="store_true", help="disable gripper trajectory command")
    p.add_argument("--overlay-yaw-offset-deg", type=float, default=0.0, help="manual yaw calibration for overlay frame")
    p.add_argument("--gripper-current-max", type=float, default=50.0, help="panel full-scale for GRIP (0..max, abs current)")
    p.add_argument("--current-log-file", default="", help="CSV log path for runtime currents (default: auto under ~/.codex/memories)")
    p.add_argument("--camera-ready-timeout", type=float, default=4.0, help="camera ready timeout per attempt (sec)")
    p.add_argument("--eef-precompute", action="store_true", help="enable startup FK precompute for EEF vector overlay")
    p.add_argument("--eef-cache-file", default="", help="path to EEF xyz cache .npy (used with --eef-precompute)")
    p.add_argument("--eef-cache-only", action="store_true", help="build EEF cache then exit (no replay)")
    p.add_argument("--no-hz-test", action="store_true", help="disable in-window hz test overlay")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    cv2_mod = _import_cv2()

    camera_ids = [int(x.strip()) for x in args.camera_ids.split(",") if x.strip()]
    if not camera_ids:
        raise RuntimeError("--camera-ids is empty")
    cam_id = camera_ids[0]

    if args.jsonl_file.strip():
        states = load_local_jsonl_episode(args.jsonl_file.strip())
        print(f"[INFO] jsonl: {args.jsonl_file.strip()} -> {states.shape[0]} steps, dim={states.shape[1]}")
    else:
        if args.episode_id is None:
            raise RuntimeError("episode_id is required when --jsonl-file is not set")
        states = load_remote_episode(args.remote_user, args.remote_host, args.remote_dataset_dir, args.episode_id)
        print(f"[INFO] episode_{args.episode_id}: {states.shape[0]} steps, dim={states.shape[1]}")

    stream = None
    if (not args.eef_cache_only) and (not args.no_camera):
        for attempt in range(1, 4):
            stream = CameraStream(cv2_mod, cam_id, 640, 480)
            if not stream.cap.isOpened():
                stream.stop()
                stream = None
                time.sleep(0.3)
                continue
            print(f"[INFO] camera /dev/video{cam_id} opened (attempt {attempt})")
            try:
                wait_camera_ready(stream, timeout_sec=float(args.camera_ready_timeout))
                print("[INFO] camera ready")
                break
            except Exception:
                stream.stop()
                stream = None
                time.sleep(0.5)
        if (not args.no_camera) and stream is None:
            raise RuntimeError(f"camera /dev/video{cam_id} open failed")

    arm = None
    current_log_file = args.current_log_file.strip()
    if not current_log_file:
        ts = time.strftime("%Y%m%d_%H%M%S")
        episode_tag = f"episode_{args.episode_id}" if args.episode_id is not None else "jsonl"
        current_log_file = os.path.expanduser(f"~/.codex/memories/replay_current_logs/{episode_tag}_{ts}.csv")
    joint_bar_maxima = FIXED_JOINT_BAR_MAXIMA.copy()
    print(
        "[INFO] fixed joint bar max: "
        + ", ".join(f"J{k+1}={joint_bar_maxima[k]:.3f}" for k in range(6))
    )
    eef_cache_file = args.eef_cache_file.strip()
    if args.jsonl_file.strip() and not eef_cache_file:
        src = args.jsonl_file.strip()
        if src.endswith(".jsonl"):
            eef_cache_file = src[:-6] + "_eef_xyz.npy"
        else:
            eef_cache_file = src + "_eef_xyz.npy"

    try:
        if not args.no_arm:
            arm = init_arm(args.arm_ip)
            if not prepare_arm_for_replay(arm):
                raise RuntimeError("arm preflight failed: cannot execute servo command")
            if args.eef_cache_only:
                if not eef_cache_file:
                    raise RuntimeError("eef cache file path is empty")
                print("[INFO] building EEF cache only ...")
                eef_xyz = load_eef_cache(eef_cache_file, expect_steps=len(states))
                if eef_xyz is not None:
                    print(f"[INFO] EEF cache already exists: {eef_cache_file}")
                else:
                    eef_xyz = precompute_eef_xyz_from_states(arm, states)
                    if eef_xyz is None:
                        raise RuntimeError("EEF precompute failed")
                    save_eef_cache(eef_cache_file, eef_xyz)
                    print(f"[INFO] EEF cache saved: {eef_cache_file}")
                return 0
            goto_init(arm, states[0])

        replay_realtime(
            cv2_mod,
            states,
            stream,
            hz=args.hz,
            arm=arm,
            gripper_current_reg=args.gripper_current_reg,
            send_gripper_cmd=(not args.no_gripper_cmd),
            overlay_yaw_offset_deg=args.overlay_yaw_offset_deg,
            gripper_current_max=args.gripper_current_max,
            current_log_file=current_log_file,
            joint_bar_maxima=joint_bar_maxima,
            eef_precompute=bool(args.eef_precompute),
            eef_cache_file=eef_cache_file,
            show_hz_test=(not args.no_hz_test),
        )
    finally:
        if stream is not None:
            stream.stop()
        if arm is not None:
            arm.disconnect()
        print("[INFO] done")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # noqa: BLE001
        print(f"[ERROR] {exc}", file=sys.stderr)
        raise SystemExit(1)
