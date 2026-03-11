#!/usr/bin/env python3
"""检测 Joy-Con 手柄摇杆向前推杆信号的测试脚本"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'joyconrobotics'))

from joyconrobotics.device import get_R_id, get_L_id
from joyconrobotics.joycon import JoyCon

FORWARD_THRESHOLD = 4000
BACKWARD_THRESHOLD = 1000

def main():
    print("=" * 50)
    print("  Joy-Con 摇杆前推信号检测工具")
    print("=" * 50)

    r_id = get_R_id()
    l_id = get_L_id()

    joycon = None
    side = None

    if r_id and r_id[0] is not None:
        print(f"\n检测到右手 Joy-Con: {r_id}")
        try:
            joycon = JoyCon(*r_id)
            side = "right"
            print("已连接右手 Joy-Con")
        except Exception as e:
            print(f"连接右手 Joy-Con 失败: {e}")

    if joycon is None:
        if l_id and l_id[0] is not None:
            print(f"\n检测到左手 Joy-Con: {l_id}")
            try:
                joycon = JoyCon(*l_id)
                side = "left"
                print("已连接左手 Joy-Con")
            except Exception as e:
                print(f"连接左手 Joy-Con 失败: {e}")

    if joycon is None:
        print("\n未检测到任何 Joy-Con 手柄，请确认：")
        print("  1. 手柄已通过蓝牙连接")
        print("  2. 有正确的 HID 读取权限")
        sys.exit(1)

    print(f"\n开始监听 {side} Joy-Con 摇杆信号...")
    print(f"  向前推: vertical > {FORWARD_THRESHOLD}")
    print(f"  向后拉: vertical < {BACKWARD_THRESHOLD}")
    print("  按 Ctrl+C 退出\n")

    time.sleep(0.5)

    try:
        while True:
            if side == "right":
                v = joycon.get_stick_right_vertical()
                h = joycon.get_stick_right_horizontal()
            else:
                v = joycon.get_stick_left_vertical()
                h = joycon.get_stick_left_horizontal()

            if v > FORWARD_THRESHOLD:
                tag = "\033[32m[前推]\033[0m"
            elif v < BACKWARD_THRESHOLD:
                tag = "\033[31m[后拉]\033[0m"
            else:
                tag = "[居中]"

            print(f"\r  vertical={v:5d}  horizontal={h:5d}  {tag}    ", end="", flush=True)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n\n已退出检测。")


if __name__ == "__main__":
    main()
