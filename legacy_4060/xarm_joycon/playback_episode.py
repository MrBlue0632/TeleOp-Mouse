#!/usr/bin/env python3
import sys, os, json, time, argparse
import numpy as np
import cv2

sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), "xArm-Python-SDK"))
from xarm.wrapper import XArmAPI

DATASET_DIR = "/home/bozhao_4060_2/ros_ws_zyl/dataset"
ARM_IP = "192.168.1.199"
DEFAULT_HZ = 30
CAM1_DEV = 4
CAM2_DEV = 10
DISP_W = 640
DISP_H = 480

JOINT_LIMITS = {
    0: (np.deg2rad(-360), np.deg2rad(360)),
    1: (np.deg2rad(-117), np.deg2rad(116)),
    2: (np.deg2rad(-219), np.deg2rad(10)),
    3: (np.deg2rad(100), np.deg2rad(283)),
    4: (np.deg2rad(-20), np.deg2rad(180)),
    5: (np.deg2rad(-100), np.deg2rad(80)),
}

def load_episode(episode_id):
    path = os.path.join(DATASET_DIR, "episode_{}_state.json".format(episode_id))
    if not os.path.exists(path):
        sys.exit("[ERROR] data not found: " + path)
    with open(path) as f:
        raw = json.load(f)
    arr = np.array(raw, dtype=np.float64)
    print("[INFO] episode_{}: {} steps".format(episode_id, arr.shape[0]))
    return arr

def clamp_rad(angles):
    out = angles.copy()
    for i, (lo, hi) in JOINT_LIMITS.items():
        if out[i] < lo or out[i] > hi:
            print("[WARN] J{} {:.2f} deg -> clamp".format(i+1, np.rad2deg(out[i])))
            out[i] = np.clip(out[i], lo, hi)
    return out

def unwrap_clamp(curr, target):
    delta = target - curr
    delta = (delta + np.pi) % (2 * np.pi) - np.pi
    return clamp_rad(curr + delta)

def init_arm():
    print("[INFO] connecting arm {}...".format(ARM_IP))
    arm = XArmAPI(ARM_IP)
    arm.motion_enable(enable=True)
    arm.set_gripper_enable(enable=True)
    arm.clean_error()
    arm.clean_warn()
    time.sleep(0.3)
    return arm

def goto_init(arm, state0, spd=20.0):
    safe_deg = list(np.rad2deg(clamp_rad(np.deg2rad(state0[:6]))))
    grip = float(state0[6])
    print("[INFO] mode=0 move to init pos, speed={} deg/s".format(spd))
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.2)
    ret = arm.set_servo_angle(angle=safe_deg, speed=spd, is_radian=False, wait=True)
    if ret != 0:
        print("[WARN] set_servo_angle returned {}".format(ret))
    arm.set_gripper_position(pos=grip, wait=True)
    print("[INFO] reached init pos, gripper={:.0f}".format(grip))
    print("[INFO] switching to mode=6 servo mode...")
    arm.set_mode(6)
    arm.set_state(0)
    time.sleep(0.5)

def open_cameras():
    c1 = cv2.VideoCapture(CAM1_DEV, cv2.CAP_V4L2)
    c2 = cv2.VideoCapture(CAM2_DEV, cv2.CAP_V4L2)
    for dev, cam, name in [(CAM1_DEV, c1, "wrist"), (CAM2_DEV, c2, "top")]:
        status = "opened" if cam.isOpened() else "FAILED"
        print("[INFO] {} cam /dev/video{} {}".format(name, dev, status))
    return c1, c2

def grab(cap, label):
    ok, frame = cap.read()
    if ok and frame is not None:
        return cv2.resize(frame, (DISP_W, DISP_H))
    ph = np.zeros((DISP_H, DISP_W, 3), dtype=np.uint8)
    cv2.putText(ph, label + " N/A", (160, 240), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 200), 2)
    return ph

def playback(arm, states, c1, c2, hz):
    n = len(states)
    dt = 1.0 / hz
    cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera View", DISP_W * 2, DISP_H)
    if arm is not None:
        _, raw = arm.get_servo_angle(is_radian=True)
        curr = np.array(raw[:6], dtype=np.float64)
    else:
        curr = np.deg2rad(states[0, :6])
    print("[INFO] playback {} steps @ {:.0f} Hz, est {:.1f} s".format(n, hz, n/hz))
    for i, state in enumerate(states):
        t0 = time.time()
        target = unwrap_clamp(curr, np.deg2rad(state[:6]))
        grip = float(state[6])
        if arm is not None:
            arm.set_servo_angle(angle=list(target), speed=40, is_radian=True)
            arm.set_gripper_position(pos=grip, wait=False)
        curr = target
        f1 = grab(c1, "Cam1")
        f2 = grab(c2, "Cam2")
        degs = " ".join("{:.1f}".format(d) for d in np.rad2deg(target))
        cv2.putText(f1, "Step {}/{}".format(i+1, n), (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(f1, "J:[{}]".format(degs), (8, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0,230,255), 1)
        cv2.putText(f2, "Gripper:{:.0f}".format(grip), (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
        cv2.putText(f2, "{:.0f}Hz {}/{}".format(hz, i+1, n), (8, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,230,255), 1)
        cv2.imshow("Camera View", np.hstack([f1, f2]))
        if cv2.waitKey(1) & 0xFF == 27:
            print("[INFO] ESC pressed, stopping")
            break
        rest = dt - (time.time() - t0)
        if rest > 0:
            time.sleep(rest)
    cv2.destroyAllWindows()
    print("[INFO] playback done")

def main():
    ap = argparse.ArgumentParser(description="xArm episode playback")
    ap.add_argument("episode_id", type=int, help="episode number (e.g. 1)")
    ap.add_argument("--hz", type=float, default=DEFAULT_HZ, help="playback rate Hz")
    ap.add_argument("--no-arm", action="store_true", help="skip arm connection (camera preview only)")
    args = ap.parse_args()
    states = load_episode(args.episode_id)
    cam1, cam2 = open_cameras()
    arm = None if args.no_arm else init_arm()
    try:
        if arm is not None:
            goto_init(arm, states[0], spd=20.0)
        print("[INFO] ready. press Enter to start (Ctrl+C to cancel)...")
        input()
        playback(arm, states, cam1, cam2, hz=args.hz)
    except KeyboardInterrupt:
        print("\n[INFO] interrupted")
    finally:
        cam1.release()
        cam2.release()
        if arm is not None:
            arm.disconnect()
        print("[INFO] done")

if __name__ == "__main__":
    main()
