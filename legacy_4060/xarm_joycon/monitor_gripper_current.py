#!/usr/bin/env python3
"""
实时监控 xArm 标准夹爪电流值
配合网页示教（xArm Studio）控制夹爪开合，观察电流变化

用法:
    python3 monitor_gripper_current.py [xarm_ip]
    例: python3 monitor_gripper_current.py 192.168.1.199

按 Ctrl+C 退出
"""

import sys
import os
import time
import struct

_script_dir = os.path.dirname(os.path.abspath(__file__))
_sdk_dir = os.path.join(_script_dir, "xArm-Python-SDK")
if os.path.exists(_sdk_dir):
    sys.path.insert(0, _sdk_dir)

from xarm.wrapper import XArmAPI

RATE_HZ = 20
BAR_WIDTH = 40
CURRENT_MAX = 100
EMA_ALPHA = 0.15  # 指数移动平均系数，越小越平滑（0.1~0.3 可调）


class EMAFilter:
    """指数移动平均滤波器"""
    def __init__(self, alpha=0.15):
        self._alpha = alpha
        self._value = None

    def update(self, raw):
        if self._value is None:
            self._value = float(raw)
        else:
            self._value = self._alpha * raw + (1 - self._alpha) * self._value
        return self._value

    @property
    def value(self):
        return self._value if self._value is not None else 0.0


def read_u16(arm, addr):
    ret = arm.core.gripper_modbus_r16s(addr, 1)
    if ret[0] == 0 and len(ret) >= 7:
        return 0, struct.unpack('>h', bytes(ret[5:7]))[0]
    return ret[0], None


def read_s32(arm, addr):
    ret = arm.core.gripper_modbus_r16s(addr, 2)
    if ret[0] == 0 and len(ret) >= 9:
        return 0, struct.unpack('>i', bytes(ret[5:9]))[0]
    return ret[0], None


def make_bar(value, max_val, width):
    ratio = min(abs(value) / max_val, 1.0) if max_val > 0 else 0
    filled = int(ratio * width)
    bar = "█" * filled + "░" * (width - filled)
    return bar


def main():
    ip = sys.argv[1] if len(sys.argv) >= 2 else input("请输入 xArm IP 地址 [192.168.1.199]: ").strip()
    if not ip:
        ip = "192.168.1.199"

    print(f"连接 xArm: {ip}")
    arm = XArmAPI(ip)
    if not arm.connected:
        print("连接失败")
        sys.exit(1)

    dt = 1.0 / RATE_HZ

    print()
    print("╔══════════════════════════════════════════════════════════╗")
    print("║     xArm 夹爪电流实时监控                               ║")
    print(f"║     刷新率: {RATE_HZ}Hz | EMA 滤波 α={EMA_ALPHA}                       ║")
    print("║     请在网页端操作夹爪开合，观察电流变化                 ║")
    print("║     Ctrl+C 退出                                         ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()

    ema = EMAFilter(alpha=EMA_ALPHA)
    peak_raw = 0
    peak_filtered = 0.0
    start_time = time.monotonic()

    try:
        while arm.connected:
            t0 = time.monotonic()

            _, current_raw = read_u16(arm, 0x0003)
            _, position = read_s32(arm, 0x0702)
            _, target_torque = read_u16(arm, 0x050A)

            if current_raw is None:
                print("\r  读取失败，重试中...                                          ", end="", flush=True)
                time.sleep(0.5)
                continue

            filtered = ema.update(current_raw)
            filtered_amp = filtered / 100.0
            raw_amp = current_raw / 100.0
            peak_raw = max(peak_raw, abs(current_raw))
            peak_filtered = max(peak_filtered, filtered)

            elapsed = time.monotonic() - start_time
            pos_str = str(position) if position is not None else "?"
            torq_str = str(target_torque) if target_torque is not None else "?"

            bar = make_bar(filtered, CURRENT_MAX, BAR_WIDTH)

            line = (
                f"\r  [{elapsed:6.1f}s]"
                f"  原始:{raw_amp:>5.2f}A"
                f"  滤波:{filtered_amp:>5.2f}A"
                f"  |{bar}|"
                f"  峰值:{peak_filtered/100:.2f}A"
                f"  位置:{pos_str:>5s}"
                f"  力矩:{torq_str:>3s}"
                f"    "
            )
            print(line, end="", flush=True)

            elapsed_loop = time.monotonic() - t0
            sleep_time = max(0, dt - elapsed_loop)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n")
        print(f"  监控结束。")
        print(f"  原始峰值: {peak_raw} (≈{peak_raw/100:.2f}A)")
        print(f"  滤波峰值: {peak_filtered:.1f} (≈{peak_filtered/100:.2f}A)")

    arm.disconnect()
    print("  已断开连接。")


if __name__ == "__main__":
    main()
