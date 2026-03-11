#!/usr/bin/env python3
"""
检测 xArm 末端夹爪型号
适用于 UFACTORY 官方夹爪：xArm 标准夹爪、BIO Gripper、Robotiq 2F-85/2F-140
"""

import sys
import os
import time

# 添加 xArm-Python-SDK 路径（与项目内其他脚本一致）
_script_dir = os.path.dirname(os.path.abspath(__file__))
_sdk_dir = os.path.join(_script_dir, "xArm-Python-SDK")
if os.path.exists(_sdk_dir):
    sys.path.insert(0, _sdk_dir)

from xarm.wrapper import XArmAPI

# 夹爪类型映射（来自 set_collision_tool_model）
GRIPPER_TYPE_MAP = {
    0: "无末端执行器",
    1: "xArm 标准夹爪 (xArm Gripper)",
    2: "xArm 真空吸盘 (Vacuum Gripper)",
    3: "xArm BIO 仿生夹爪 (BIO Gripper)",
    4: "Robotiq 2F-85 夹爪",
    5: "Robotiq 2F-140 夹爪",
}


def detect_by_collision_config(arm):
    """通过自碰撞检测配置读取夹爪类型（若用户已配置）"""
    try:
        params = arm.self_collision_params
        tool_type = params[1] if len(params) > 1 else 0
        return tool_type, GRIPPER_TYPE_MAP.get(tool_type, f"未知类型({tool_type})")
    except Exception as e:
        return None, str(e)


def try_xarm_gripper(arm):
    """尝试 xArm 标准夹爪 API"""
    try:
        code, pos = arm.get_gripper_position()
        if code == 0 and 0 <= pos <= 1000:  # xArm 夹爪位置范围
            return True, f"get_gripper_position 成功, 位置={pos}"
        return False, f"code={code}, pos={pos}"
    except Exception as e:
        return False, str(e)


def try_bio_gripper(arm):
    """尝试 BIO 夹爪 API"""
    try:
        code, status = arm.get_bio_gripper_status()
        if code == 0:
            return True, f"get_bio_gripper_status 成功, status={status}"
        return False, f"code={code}"
    except Exception as e:
        return False, str(e)


def try_robotiq_gripper(arm):
    """尝试 Robotiq 夹爪 API"""
    try:
        code, ret = arm.robotiq_get_status(number_of_registers=3)
        if code == 0 and len(ret) > 0:
            return True, f"robotiq_get_status 成功, gSTA={arm.robotiq_status.get('gSTA', '?')}"
        return False, f"code={code}"
    except Exception as e:
        return False, str(e)


def main():
    ip = sys.argv[1] if len(sys.argv) >= 2 else input("请输入 xArm IP 地址: ")
    if not ip:
        print("未输入 IP，退出")
        sys.exit(1)

    print("=" * 50)
    print("xArm 夹爪型号检测")
    print("=" * 50)

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

    # 1. 检查碰撞配置中的夹爪类型
    print("\n[1] 自碰撞配置中的夹爪类型:")
    tool_type, type_name = detect_by_collision_config(arm)
    if tool_type is not None:
        print(f"    collision_tool_type = {tool_type}")
        print(f"    对应: {type_name}")
        print("    (注: 此为用户配置值，可能与实际硬件不一致)")
    else:
        print(f"    读取失败: {type_name}")

    # 2. 依次尝试各夹爪 API
    print("\n[2] 尝试各夹爪 API 检测实际硬件:")
    detected = []

    print("    尝试 xArm 标准夹爪 (get_gripper_position)...")
    ok, msg = try_xarm_gripper(arm)
    if ok:
        detected.append(("xArm 标准夹爪", msg))
        print(f"    ✓ 成功: {msg}")
    else:
        print(f"    ✗ 失败: {msg}")

    print("    尝试 BIO 夹爪 (get_bio_gripper_status)...")
    ok, msg = try_bio_gripper(arm)
    if ok:
        detected.append(("xArm BIO 仿生夹爪", msg))
        print(f"    ✓ 成功: {msg}")
    else:
        print(f"    ✗ 失败: {msg}")

    print("    尝试 Robotiq 夹爪 (robotiq_get_status)...")
    ok, msg = try_robotiq_gripper(arm)
    if ok:
        detected.append(("Robotiq 夹爪", msg))
        print(f"    ✓ 成功: {msg}")
    else:
        print(f"    ✗ 失败: {msg}")

    # 3. 结论
    print("\n" + "=" * 50)
    if len(detected) == 1:
        print(f"检测结果: 您的夹爪是 【{detected[0][0]}】")
    elif len(detected) > 1:
        print("检测到多个夹爪响应，可能配置异常。响应列表:")
        for name, msg in detected:
            print(f"  - {name}: {msg}")
    else:
        print("未能通过 API 检测到夹爪类型，可能原因:")
        print("  - 夹爪未连接或未上电")
        print("  - 夹爪型号不在支持列表（xArm/BIO/Robotiq）")
        print("  - 通信波特率或接线有误")
    print("=" * 50)

    arm.disconnect()


if __name__ == "__main__":
    main()
