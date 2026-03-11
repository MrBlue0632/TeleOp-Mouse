#!/usr/bin/env python3
"""
实验性脚本：通过 Modbus 读取 xArm 标准夹爪的寄存器
目的：探索夹爪是否支持电流/力矩等力相关信号的读取

阶段1: 扫描所有已知寄存器，打印原始值
阶段2: 对成功读取的寄存器进入实时监控循环（可在夹爪开合时观察数值变化）

用法:
    python3 read_gripper_registers.py [xarm_ip]
    例: python3 read_gripper_registers.py 192.168.1.199
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

GRIPPER_REGISTERS = [
    # (地址, 读取长度(16位), 名称, 解析方式, 说明)
    # 解析方式: 'u16' / 's16' / 'u32' / 's32'
    (0x0000, 1, "状态寄存器",     "u16",  "夹爪状态"),
    (0x0001, 1, "CURR_CURR",      "s16",  "电流（转速相关）"),
    (0x0002, 1, "电流百分比",     "s16",  "电流百分比"),
    (0x0003, 1, "电流值",         "s16",  "电流（/100 得到实际值）"),
    (0x000B, 1, "电角度",         "s16",  "电角度"),
    (0x000E, 1, "温度",           "u16",  "温度 (°C)"),
    (0x000F, 1, "错误码",         "u16",  "ERR_CODE"),
    (0x0100, 1, "使能状态",       "u16",  "CON_EN"),
    (0x0101, 1, "模式",           "u16",  "CON_MODE"),
    (0x0303, 1, "速度",           "u16",  "POS_SPD"),
    (0x050a, 1, "目标力矩",       "s16",  "TAGET_TOQ"),
    (0x050c, 1, "当前力矩",       "s16",  "CURR_TOQ"),
    (0x050e, 1, "力矩速度",       "s16",  "TOQ_SPD"),
    (0x0700, 2, "目标位置",       "s32",  "TAGET_POS"),
    (0x0702, 2, "当前位置",       "s32",  "CURR_POS"),
    (0x0800, 1, "硬件版本",       "u16",  "HARD_VER"),
    (0x0801, 1, "软件版本-主",    "u16",  "SOFT_VER major"),
    (0x0802, 1, "电机类型",       "u16",  "MT_TYPE"),
]


def parse_value(raw_bytes, fmt):
    """根据格式解析原始字节（大端序）"""
    if fmt == "u16" and len(raw_bytes) >= 2:
        return raw_bytes[0] << 8 | raw_bytes[1]
    elif fmt == "s16" and len(raw_bytes) >= 2:
        return struct.unpack('>h', bytes(raw_bytes[:2]))[0]
    elif fmt == "u32" and len(raw_bytes) >= 4:
        return (raw_bytes[0] << 24 | raw_bytes[1] << 16
                | raw_bytes[2] << 8 | raw_bytes[3])
    elif fmt == "s32" and len(raw_bytes) >= 4:
        return struct.unpack('>i', bytes(raw_bytes[:4]))[0]
    return None


def read_register(arm, addr, length):
    """
    读取夹爪 Modbus 寄存器
    返回 (code, raw_bytes)
    code=0 成功, raw_bytes 为数据部分（不含前5字节协议头）
    """
    ret = arm.core.gripper_modbus_r16s(addr, length)
    code = ret[0]
    expected_len = 5 + length * 2
    if code == 0 and len(ret) >= expected_len:
        return 0, ret[5:expected_len]
    return code if code != 0 else -1, []


def scan_registers(arm):
    """阶段1：扫描所有寄存器"""
    print("=" * 70)
    print("阶段1: 扫描夹爪 Modbus 寄存器")
    print("=" * 70)
    print(f"{'地址':>8s}  {'名称':<14s}  {'状态码':>4s}  {'原始值':>10s}  {'说明'}")
    print("-" * 70)

    success_list = []

    for addr, length, name, fmt, desc in GRIPPER_REGISTERS:
        addr_str = f"0x{addr:04X}"
        code, raw = read_register(arm, addr, length)
        if code == 0:
            val = parse_value(raw, fmt)
            raw_hex = " ".join(f"{b:02X}" for b in raw)
            extra = ""
            if addr == 0x0003 and val is not None:
                extra = f"  (≈ {val / 100:.2f} A)"
            print(f"{addr_str:>8s}  {name:<14s}  OK    {val:>10}  {desc}{extra}  [{raw_hex}]")
            success_list.append((addr, length, name, fmt, desc))
        else:
            print(f"{addr_str:>8s}  {name:<14s}  ERR={code:<3}  {'---':>10s}  {desc}")

    return success_list


FORCE_RELATED_ADDRS = {0x0001, 0x0002, 0x0003, 0x050a, 0x050c, 0x050e}


def monitor_registers(arm, reg_list):
    """阶段2：实时监控成功读取的寄存器（重点关注力/电流相关）"""
    monitor_regs = [r for r in reg_list if r[0] in FORCE_RELATED_ADDRS]
    extra_regs = [r for r in reg_list if r[0] == 0x0702]
    monitor_regs.extend(extra_regs)

    if not monitor_regs:
        print("\n没有可监控的力/电流相关寄存器，跳过实时监控。")
        return

    print("\n" + "=" * 70)
    print("阶段2: 实时监控力/电流相关寄存器 (Ctrl+C 退出)")
    print("  提示: 请尝试开合夹爪，观察数值变化")
    print("=" * 70)

    header = "  ".join(f"{r[2]:>10s}" for r in monitor_regs)
    print(f"{'时间':>8s}  {header}")
    print("-" * (10 + len(monitor_regs) * 12))

    start = time.monotonic()
    try:
        while arm.connected:
            elapsed = time.monotonic() - start
            values = []
            for addr, length, name, fmt, desc in monitor_regs:
                code, raw = read_register(arm, addr, length)
                if code == 0:
                    val = parse_value(raw, fmt)
                    values.append(f"{val:>10}" if val is not None else f"{'???':>10s}")
                else:
                    values.append(f"{'ERR':>10s}")
            line = "  ".join(values)
            print(f"{elapsed:>7.1f}s  {line}")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n监控已停止。")


def main():
    ip = sys.argv[1] if len(sys.argv) >= 2 else input("请输入 xArm IP 地址 [192.168.1.199]: ").strip()
    if not ip:
        ip = "192.168.1.199"

    print(f"连接 xArm: {ip}")
    arm = XArmAPI(ip)
    if not arm.connected:
        print("连接失败，请检查 IP 和网络")
        sys.exit(1)

    arm.motion_enable(enable=True)
    arm.clean_error()
    arm.clean_warn()
    arm.set_mode(0)
    arm.set_state(0)
    time.sleep(0.3)

    print("使能夹爪...")
    code = arm.set_gripper_enable(True)
    if code != 0:
        print(f"夹爪使能失败 (code={code})，继续尝试读取寄存器...")
    time.sleep(0.3)

    success_list = scan_registers(arm)

    print(f"\n扫描完成: {len(success_list)}/{len(GRIPPER_REGISTERS)} 个寄存器可读")

    if success_list:
        monitor_registers(arm, success_list)

    arm.disconnect()
    print("已断开连接。")


if __name__ == "__main__":
    main()
