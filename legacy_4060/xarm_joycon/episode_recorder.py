#!/usr/bin/env python3
import rospy
import numpy as np
import json
import os
import time
from std_msgs.msg import Float64MultiArray
from threading import Lock
import signal
import sys
import re
import subprocess

from num2words import num2words

exit_flag = False

# 录制帧率（Hz）
RECORD_FPS = 30

# 数据存储路径
SAVE_DIR = "/home/bozhao_4060_2/ros_ws_zyl/dataset"


def handle_shutdown(signum, frame):
    global exit_flag
    exit_flag = True
    print("\n[Shutdown] Ctrl+C pressed. Cleaning up...")
    rospy.signal_shutdown("Ctrl+C exit")

signal.signal(signal.SIGINT, handle_shutdown)


def speak(text: str):
    try:
        def replace_numbers(match):
            return num2words(int(match.group()))
        text = re.sub(r'\b\d+\b', replace_numbers, text)
        subprocess.run(["espeak-ng", "-v", "en", "-s", "150", text])
    except Exception as e:
        rospy.logwarn(f"Speech failed: {e}")


class Recorder:
    def __init__(self, record_fps=120):
        self.lock = Lock()
        self.latest_state = None
        self.episode = []
        self.recording = False
        self.record_fps = record_fps
        self.last_record_time = 0.0

        rospy.Subscriber('/robot_state', Float64MultiArray, self.state_callback)

    def state_callback(self, msg):
        with self.lock:
            self.latest_state = np.array(msg.data, dtype=np.float64)
            self.try_record()

    def try_record(self):
        if not self.recording:
            return

        if self.latest_state is None:
            return

        now = time.time()
        min_interval = 1.0 / self.record_fps if self.record_fps > 0 else 0
        if min_interval > 0 and (now - self.last_record_time) < min_interval:
            return
        self.last_record_time = now

        self.episode.append(self.latest_state.copy())
        rospy.loginfo(f"Recorded step {len(self.episode)}")
        self.latest_state = None

    def save_episode(self, episode_id: int):
        if not self.episode:
            rospy.logwarn("No data recorded, skipping save.")
            return

        os.makedirs(SAVE_DIR, exist_ok=True)
        states = [step.tolist() for step in self.episode]
        save_path = os.path.join(SAVE_DIR, f"episode_{episode_id}_state.json")
        with open(save_path, 'w') as f:
            json.dump(states, f)
        rospy.loginfo(f"✅ Episode {episode_id} saved to {save_path}  (steps: {len(states)}, dim: {len(states[0])})")
        self.episode = []


def get_next_episode_id(save_dir: str) -> int:
    if not os.path.exists(save_dir):
        return 1
    existing = [f for f in os.listdir(save_dir) if f.startswith("episode_") and f.endswith("_state.json")]
    episode_ids = []
    for name in existing:
        try:
            num = int(name.split("_")[1])
            episode_ids.append(num)
        except:
            continue
    return max(episode_ids, default=0) + 1


if __name__ == '__main__':
    try:
        rospy.init_node('recorder')
        rospy.loginfo(f"📹 Recording FPS: {RECORD_FPS} Hz | Save dir: {SAVE_DIR}")
        recorder = Recorder(record_fps=RECORD_FPS)

        num_episodes = 7
        record_duration = 60
        rest_duration = 50
        continue_recording = True
        os.makedirs(SAVE_DIR, exist_ok=True)

        start_id = get_next_episode_id(SAVE_DIR) if continue_recording else 1

        for episode_id in range(start_id, start_id + num_episodes):
            if exit_flag:
                break

            msg = f"recording episode {episode_id}"
            rospy.loginfo(f"🔴 {msg}, duration {record_duration}s...")
            speak(msg)

            recorder.recording = True
            start_time = time.time()
            rate = rospy.Rate(10)
            while time.time() - start_time < record_duration and not rospy.is_shutdown() and not exit_flag:
                key_check = False
                rate.sleep()

            recorder.recording = False

            if exit_flag:
                break

            speak(f"recording completed, saving episode {episode_id}")
            recorder.save_episode(episode_id)

            rospy.loginfo(f"✅ Episode {episode_id} saved. Resting for {rest_duration}s...\n")
            speak(f"rest for {rest_duration} seconds")

            rest_start = time.time()
            while time.time() - rest_start < rest_duration and not exit_flag:
                rate.sleep()

        speak("all episodes recorded and saved")
        rospy.loginfo("🎉 All episodes completed.")

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt received. Shutdown.")
