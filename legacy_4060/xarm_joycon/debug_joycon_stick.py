#!/usr/bin/env python3
"""
JoyCon 摇杆快速诊断脚本 - 排查前推摇杆无响应问题
总运行时间约 5 秒，逐行输出结果，不会卡住。
用法: python3 debug_joycon_stick.py
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "joyconrobotics"))

THRESHOLD_HIGH = 3750
THRESHOLD_LOW  = 1000

def main():
    print("=" * 60)
    print("  JoyCon 摇杆快速诊断")
    print("=" * 60)

    # ── 1. HID 设备扫描 ──
    print("\n[步骤1] HID 设备扫描...")
    try:
        import hid
    except ImportError:
        print("  FAIL: 无法导入 hid 模块，请: pip install hidapi")
        return 1

    devices = hid.enumerate(0, 0)
    joycon_devs = [d for d in devices if d["vendor_id"] == 0x057E and d["product_id"] in (0x2006, 0x2007)]
    if not joycon_devs:
        print("  FAIL: 未检测到 JoyCon HID 设备!")
        print("  请确认蓝牙已连接 + 有 hidraw 读写权限")
        return 1

    for d in joycon_devs:
        side = "R(右)" if d["product_id"] == 0x2007 else "L(左)"
        serial = d.get("serial") or d.get("serial_number") or "?"
        print(f"  OK: {side}  vid=0x{d['vendor_id']:04X} pid=0x{d['product_id']:04X} serial={serial}")

    # ── 2. 获取设备 ID ──
    print("\n[步骤2] 获取设备 ID...")
    from joyconrobotics.device import get_R_id, get_L_id
    r_id = get_R_id()
    l_id = get_L_id()
    print(f"  get_R_id() = {r_id}")
    print(f"  get_L_id() = {l_id}")

    # 优先用右手柄，没有就用左手柄
    if r_id != (None, None, None):
        jid = r_id
        side = "right"
    elif l_id != (None, None, None):
        jid = l_id
        side = "left"
    else:
        print("  FAIL: get_R_id 和 get_L_id 都返回 None!")
        return 1

    # ── 3. 连接 JoyCon ──
    print(f"\n[步骤3] 连接 JoyCon ({side})...")
    from joyconrobotics.joycon import JoyCon
    try:
        jc = JoyCon(*jid)
    except Exception as e:
        print(f"  FAIL: 连接失败: {e}")
        return 1

    print(f"  OK: 连接成功")
    print(f"  is_right() = {jc.is_right()}")
    print(f"  is_left()  = {jc.is_left()}")

    if side == "right" and not jc.is_right():
        print("  WARN: 期望右手柄但 is_right()=False，手柄识别可能有误!")
    if side == "left" and not jc.is_left():
        print("  WARN: 期望左手柄但 is_left()=False，手柄识别可能有误!")

    is_right = jc.is_right()
    time.sleep(0.3)

    # ── 4. 快速采样 3 秒 ──
    print(f"\n[步骤4] 快速采样摇杆值 (3秒, 请期间前后左右推动摇杆)...")
    print(f"  当前代码阈值: 前推>{THRESHOLD_HIGH}, 后拉<{THRESHOLD_LOW}")

    v_min, v_max = 99999, -1
    h_min, h_max = 99999, -1
    samples = []

    start = time.time()
    count = 0
    print_interval = 0.3
    next_print = start + print_interval

    while time.time() - start < 3.0:
        if is_right:
            sv = jc.get_stick_right_vertical()
            sh = jc.get_stick_right_horizontal()
        else:
            sv = jc.get_stick_left_vertical()
            sh = jc.get_stick_left_horizontal()

        v_min = min(v_min, sv)
        v_max = max(v_max, sv)
        h_min = min(h_min, sh)
        h_max = max(h_max, sh)
        samples.append((sv, sh))
        count += 1

        now = time.time()
        if now >= next_print:
            fwd = " <<< FWD触发!" if sv > THRESHOLD_HIGH else ""
            bwd = " <<< BWD触发!" if sv < THRESHOLD_LOW else ""
            print(f"  V={sv:5d}  H={sh:5d}{fwd}{bwd}")
            next_print = now + print_interval

        time.sleep(0.005)

    # 取前20个样本估算中心值
    center_samples = samples[:min(20, len(samples))]
    v_center = sum(s[0] for s in center_samples) / len(center_samples)
    h_center = sum(s[1] for s in center_samples) / len(center_samples)

    # ── 5. 原始字节检查 ──
    print(f"\n[步骤5] input_report 原始字节检查...")
    report = jc._input_report
    b6, b7, b8  = report[6], report[7], report[8]
    b9, b10, b11 = report[9], report[10], report[11]
    r_h_raw = b9 | ((b10 & 0x0F) << 8)
    r_v_raw = ((b10 >> 4) & 0x0F) | (b11 << 4)
    l_h_raw = b6 | ((b7 & 0x0F) << 8)
    l_v_raw = ((b7 >> 4) & 0x0F) | (b8 << 4)
    print(f"  原始字节 [6:12] = [{b6},{b7},{b8},{b9},{b10},{b11}]")
    print(f"  解析: L(h={l_h_raw}, v={l_v_raw})  R(h={r_h_raw}, v={r_v_raw})")

    # ── 6. 分析结果 ──
    print("\n" + "=" * 60)
    print("  诊断结果")
    print("=" * 60)
    print(f"  采样数: {count}")
    print(f"  V(前后): 中心={v_center:.0f}, 最小={v_min}, 最大={v_max}")
    print(f"  H(左右): 中心={h_center:.0f}, 最小={h_min}, 最大={h_max}")
    print()

    problem_found = False

    if v_max <= THRESHOLD_HIGH:
        print(f"  *** 问题发现: 前推最大值 {v_max} 没有达到阈值 {THRESHOLD_HIGH} ***")
        print(f"      这就是前推无响应的原因!")
        problem_found = True
        suggested = int(v_center + (v_max - v_center) * 0.7)
        print(f"      建议将阈值从 {THRESHOLD_HIGH} 降到 {suggested}")
    else:
        print(f"  OK: 前推最大值 {v_max} > 阈值 {THRESHOLD_HIGH}")

    if v_min >= THRESHOLD_LOW:
        print(f"  *** 问题发现: 后拉最小值 {v_min} 没有低于阈值 {THRESHOLD_LOW} ***")
        problem_found = True
        suggested = int(v_center - (v_center - v_min) * 0.7)
        print(f"      建议将阈值从 {THRESHOLD_LOW} 升到 {suggested}")
    else:
        print(f"  OK: 后拉最小值 {v_min} < 阈值 {THRESHOLD_LOW}")

    if h_max <= 3750:
        print(f"  WARN: 左右最大值 {h_max} 没达到 3750")
    if h_min >= 1000:
        print(f"  WARN: 左右最小值 {h_min} 没低于 1000")

    v_range = v_max - v_min
    print(f"\n  摇杆 V 总行程: {v_range} (正常应 > 3000)")
    if v_range < 500:
        print(f"  *** 行程极小，摇杆可能没有被推动，或硬件故障 ***")

    if not problem_found and v_range > 2000:
        print("\n  摇杆数据看起来正常，问题可能在:")
        print("    1. solve_loop 线程崩溃 (异常被静默吞掉)")
        print("    2. glimit 位置限制截断了 position[0]")
        print("    3. dof_speed[0] 的值为 0")
        print("    4. 上层调用没有正确读取 get_control() 的结果")

    # ── 7. 按钮快照 ──
    print(f"\n[步骤7] 按钮状态快照 (当前瞬间):")
    if is_right:
        print(f"  R={jc.get_button_r()} ZR={jc.get_button_zr()} "
              f"A={jc.get_button_a()} B={jc.get_button_b()} "
              f"X={jc.get_button_x()} Y={jc.get_button_y()} "
              f"Plus={jc.get_button_plus()} Home={jc.get_button_home()} "
              f"RStick={jc.get_button_r_stick()}")
    else:
        print(f"  L={jc.get_button_l()} ZL={jc.get_button_zl()} "
              f"Up={jc.get_button_up()} Down={jc.get_button_down()} "
              f"Left={jc.get_button_left()} Right={jc.get_button_right()} "
              f"Minus={jc.get_button_minus()} Capture={jc.get_button_capture()} "
              f"LStick={jc.get_button_l_stick()}")

    print("\n诊断完成。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
