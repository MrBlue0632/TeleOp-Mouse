#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass
class LatestFrame:
    frame: Any | None = None
    ts: float = 0.0
    count: int = 0


class CameraStream:
    def __init__(self, cv2_mod, cam_id: int, width: int = 640, height: int = 480):
        import threading

        self.cv2 = cv2_mod
        self.cap = cv2_mod.VideoCapture(cam_id, cv2_mod.CAP_V4L2)
        if self.cap.isOpened():
            self.cap.set(cv2_mod.CAP_PROP_FRAME_WIDTH, width)
            self.cap.set(cv2_mod.CAP_PROP_FRAME_HEIGHT, height)
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
                time.sleep(0.01)

    def get(self):
        with self._lock:
            return self._latest.frame, self._latest.ts, self._latest.count


def _import_cv2():
    import cv2  # type: ignore

    return cv2


def _import_xarm():
    from xarm.wrapper import XArmAPI  # type: ignore

    return XArmAPI


def load_episode_jsonl(path: str):
    rows = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            d = json.loads(line)
            q = d.get("joints_deg")
            g = d.get("gripper_pos", None)
            if isinstance(q, list) and len(q) >= 6:
                rows.append((np.array(q[:6], dtype=np.float64), None if g is None else float(g)))
    if not rows:
        raise RuntimeError(f"no valid joints in {path}")
    return rows


def wait_camera_ready(stream: CameraStream, timeout_sec: float = 8.0) -> None:
    stream.start()
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_sec:
        frame, ts, count = stream.get()
        if frame is not None and count >= 1 and (time.monotonic() - ts) < 1.0:
            return
        time.sleep(0.03)
    raise RuntimeError("camera not ready")


def prepare_mode6(arm, retries: int = 6) -> bool:
    for i in range(retries):
        try:
            arm.clean_error()
            arm.set_state(0)
            arm.motion_enable(True)
            arm.set_mode(6)
            arm.set_state(0)
            time.sleep(0.2)
            code, ang = arm.get_servo_angle(is_radian=True)
            if code == 0 and isinstance(ang, list):
                ret = arm.set_servo_angle(angle=list(np.array(ang[:6], dtype=float)), speed=float(np.deg2rad(20)), is_radian=True)
                if ret == 0:
                    print(f"[INFO] mode6 preflight ok (attempt {i+1})")
                    return True
        except Exception:
            pass
        time.sleep(0.3)
    return False


def open_camera_with_retry(cv2, camera_id: int, attempts: int = 3) -> CameraStream:
    last_err = None
    for i in range(1, attempts + 1):
        stream = CameraStream(cv2, camera_id, 640, 480)
        if not stream.cap.isOpened():
            stream.stop()
            time.sleep(0.3)
            continue
        try:
            wait_camera_ready(stream, timeout_sec=10.0)
            print(f"[INFO] camera /dev/video{camera_id} ready (attempt {i})")
            return stream
        except Exception as exc:
            last_err = exc
            stream.stop()
            time.sleep(0.5)
    raise RuntimeError(f"camera /dev/video{camera_id} not ready: {last_err}")


def replay(path: str, arm_ip: str, hz: float, camera_id: int):
    cv2 = _import_cv2()
    rows = load_episode_jsonl(path)
    print(f"[INFO] loaded {len(rows)} steps")

    stream = open_camera_with_retry(cv2, camera_id, attempts=3)

    XArmAPI = _import_xarm()
    arm = XArmAPI(arm_ip)
    arm.set_gripper_enable(True)
    time.sleep(0.2)

    q0, g0 = rows[0]
    arm.clean_error()
    arm.set_state(0)
    arm.motion_enable(True)
    arm.set_mode(0)
    arm.set_state(0)
    r = arm.set_servo_angle(angle=q0.tolist(), speed=20, is_radian=False, wait=False)
    print(f"[INFO] goto init ret={r}")
    if g0 is not None:
        arm.set_gripper_position(pos=max(0, min(850, g0)), wait=False)
    time.sleep(1.2)

    if not prepare_mode6(arm):
        arm.disconnect()
        stream.stop()
        raise RuntimeError("mode6 preflight failed")

    dt = 1.0 / hz
    next_t = time.monotonic()
    win = "ReplayRealtime"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    try:
        for i, (q_deg, g) in enumerate(rows):
            now = time.monotonic()
            if now < next_t:
                time.sleep(next_t - now)
            q_rad = np.deg2rad(q_deg)
            sret = arm.set_servo_angle(angle=q_rad.tolist(), speed=float(np.deg2rad(40)), is_radian=True)
            if sret != 0:
                arm.clean_error()
                arm.set_state(0)
                arm.motion_enable(True)
                arm.set_mode(6)
                arm.set_state(0)
            if g is not None and (i % 2 == 0):
                arm.set_gripper_position(pos=max(0, min(850, g)), wait=False)

            frame, _, _ = stream.get()
            if frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
            else:
                frame = cv2.resize(frame, (640, 480))
            cv2.putText(frame, f"Replay {i+1}/{len(rows)} @ {hz:.1f}Hz", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (80, 240, 240), 2)
            cv2.imshow(win, frame)
            if (cv2.waitKey(1) & 0xFF) == 27:
                break

            next_t = time.monotonic() + dt
    finally:
        cv2.destroyAllWindows()
        stream.stop()
        arm.disconnect()
        print("[INFO] replay done")


def parse_args():
    p = argparse.ArgumentParser(description="Replay Teleop jsonl with local camera")
    p.add_argument("jsonl")
    p.add_argument("--arm-ip", default="127.0.0.1")
    p.add_argument("--hz", type=float, default=30.0)
    p.add_argument("--camera-id", type=int, default=5)
    return p.parse_args()


if __name__ == "__main__":
    args = parse_args()
    replay(args.jsonl, args.arm_ip, args.hz, args.camera_id)
