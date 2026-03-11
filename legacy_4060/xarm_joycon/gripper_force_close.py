#!/usr/bin/env python3
"""
电流控制夹爪闭合：从 830 开始慢慢闭合，当电流稳定在目标值附近时停止

用法:
    python3 gripper_force_close.py [xarm_ip]
    例: python3 gripper_force_close.py 192.168.1.199

按 Ctrl+C 随时中断并停在当前位置
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

# ==================== 参数配置 ====================
START_POS = 830            # 起始位置（接近全开）
MIN_POS = 0                # 最小位置（全闭极限，安全下限）
TARGET_CURRENT_A = 0.15    # 目标电流 (A)
TOLERANCE_A = 0.03         # 容许偏差 (A)，在 target ± tolerance 内认为达标
STEP_SIZE = 15             # 每步闭合量（位置脉冲），越大闭合越快
STEP_INTERVAL = 0.05       # 每步间隔 (s)，越小闭合越快
EMA_ALPHA = 0.25           # 电流滤波系数
HOLD_CONFIRM_COUNT = 10    # 连续多少次采样在目标范围内才确认稳定
MONITOR_HZ = 20            # 保持阶段的监控频率
OVERSHOOT_A = 0.50         # 超过此电流则回退张开
RETREAT_STEP = 20          # 超过电流上限时回退的脉冲量
# ==================================================


class EMAFilter:
    def __init__(self, alpha=0.2):
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


def read_current(arm):
    """读取夹爪电流寄存器 0x0003，返回原始值（/100=安培）"""
    ret = arm.core.gripper_modbus_r16s(0x0003, 1)
    if ret[0] == 0 and len(ret) >= 7:
        return struct.unpack('>h', bytes(ret[5:7]))[0]
    return None


def read_position(arm):
    """读取夹爪当前位置寄存器 0x0702"""
    ret = arm.core.gripper_modbus_r16s(0x0702, 2)
    if ret[0] == 0 and len(ret) >= 9:
        return struct.unpack('>i', bytes(ret[5:9]))[0]
    return None


def make_bar(value, max_val, width=30):
    ratio = min(abs(value) / max_val, 1.0) if max_val > 0 else 0
    filled = int(ratio * width)
    return "█" * filled + "░" * (width - filled)


def main():
    ip = sys.argv[1] if len(sys.argv) >= 2 else input("请输入 xArm IP [192.168.1.199]: ").strip()
    if not ip:
        ip = "192.168.1.199"

    target_raw = TARGET_CURRENT_A * 100
    tolerance_raw = TOLERANCE_A * 100
    overshoot_raw = OVERSHOOT_A * 100

    print(f"连接 xArm: {ip}")
    arm = XArmAPI(ip)
    if not arm.connected:
        print("连接失败")
        sys.exit(1)

    arm.motion_enable(enable=True)
    arm.clean_error()
    arm.clean_warn()
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.3)

    arm.set_gripper_enable(True)
    time.sleep(0.2)

    print()
    print("╔══════════════════════════════════════════════════════════╗")
    print(f"║  夹爪电流控制闭合                                        ║")
    print(f"║  起始: {START_POS}  目标电流: {TARGET_CURRENT_A}A ± {TOLERANCE_A}A                ║")
    print(f"║  步长: {STEP_SIZE}  间隔: {STEP_INTERVAL}s  超限回退: {RETREAT_STEP}                   ║")
    print(f"║  电流 > {OVERSHOOT_A}A 时自动回退张开                            ║")
    print(f"║  Ctrl+C 随时停止                                         ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()

    # 先移到起始位置
    print(f"  移动到起始位置 {START_POS} ...")
    arm.set_gripper_position(START_POS, wait=True, speed=3000)
    time.sleep(0.5)

    ema = EMAFilter(alpha=EMA_ALPHA)
    # 预热滤波器
    for _ in range(10):
        raw = read_current(arm)
        if raw is not None:
            ema.update(raw)
        time.sleep(0.02)

    current_pos = START_POS
    in_range_count = 0
    reached = False

    print("  开始闭合...\n")
    print(f"  {'位置':>6s}  {'原始':>6s}  {'滤波':>6s}  {'目标':>6s}  {'状态'}")
    print("  " + "-" * 60)

    try:
        # === 阶段1: 逐步闭合，超限回退，直到电流达标 ===
        while arm.connected and current_pos > MIN_POS:
            current_pos -= STEP_SIZE
            current_pos = max(current_pos, MIN_POS)
            arm.set_gripper_position(current_pos, wait=False)

            time.sleep(STEP_INTERVAL)

            raw = read_current(arm)
            if raw is None:
                print(f"  {current_pos:>6d}  {'ERR':>6s}  {'---':>6s}  {TARGET_CURRENT_A:>5.2f}A  读取失败")
                continue

            filtered = ema.update(raw)
            filtered_a = filtered / 100.0
            raw_a = raw / 100.0
            bar = make_bar(filtered, target_raw + tolerance_raw * 3, 20)

            if filtered > overshoot_raw:
                current_pos = min(current_pos + RETREAT_STEP, START_POS)
                arm.set_gripper_position(current_pos, wait=False)
                in_range_count = 0
                status = f"⚠ 超限回退 -> {current_pos}"
            elif abs(filtered - target_raw) <= tolerance_raw:
                in_range_count += 1
                status = f"达标中 ({in_range_count}/{HOLD_CONFIRM_COUNT})"
            elif filtered > target_raw + tolerance_raw:
                current_pos = min(current_pos + RETREAT_STEP // 2, START_POS)
                arm.set_gripper_position(current_pos, wait=False)
                in_range_count = 0
                status = f"偏高回退 -> {current_pos}"
            else:
                in_range_count = 0
                status = f"|{bar}|"

            print(f"  {current_pos:>6d}  {raw_a:>5.2f}A  {filtered_a:>5.2f}A  {TARGET_CURRENT_A:>5.2f}A  {status}")

            if in_range_count >= HOLD_CONFIRM_COUNT:
                reached = True
                break

        if current_pos <= MIN_POS and not reached:
            print(f"\n  已到达最小位置 {MIN_POS}，未达到目标电流")

        if reached:
            actual_pos = read_position(arm) or current_pos
            print(f"\n  ✓ 电流已稳定在 {TARGET_CURRENT_A}A 附近！")
            print(f"    停止位置: {actual_pos}")
            print(f"    滤波电流: {ema.value/100:.2f}A")
            print(f"\n  进入保持监控 (电流超限自动回退)... Ctrl+C 退出\n")

            # === 阶段2: 保持并监控，超限自动回退 ===
            hold_pos = current_pos
            dt = 1.0 / MONITOR_HZ
            while arm.connected:
                t0 = time.monotonic()
                raw = read_current(arm)
                pos = read_position(arm)
                action = ""
                if raw is not None:
                    filtered = ema.update(raw)
                    filtered_a = filtered / 100.0
                    raw_a = raw / 100.0
                    pos_str = str(pos) if pos is not None else "?"
                    bar = make_bar(filtered, 50, 30)

                    if filtered > overshoot_raw:
                        hold_pos = min(hold_pos + RETREAT_STEP, START_POS)
                        arm.set_gripper_position(hold_pos, wait=False)
                        action = " ⚠回退"
                    elif filtered > target_raw + tolerance_raw:
                        hold_pos = min(hold_pos + RETREAT_STEP // 2, START_POS)
                        arm.set_gripper_position(hold_pos, wait=False)
                        action = " ↑微退"

                    print(
                        f"\r  保持中  原始:{raw_a:>5.2f}A  滤波:{filtered_a:>5.2f}A"
                        f"  |{bar}|  位置:{pos_str:>5s}{action:6s}    ",
                        end="", flush=True
                    )
                elapsed = time.monotonic() - t0
                time.sleep(max(0, dt - elapsed))

    except KeyboardInterrupt:
        print("\n\n  用户中断。")

    final_pos = read_position(arm) or current_pos
    final_current = ema.value / 100.0
    print(f"\n  最终位置: {final_pos}, 滤波电流: {final_current:.2f}A")

    arm.disconnect()
    print("  已断开连接。")


if __name__ == "__main__":
    main()
