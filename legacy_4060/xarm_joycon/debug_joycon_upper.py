#!/usr/bin/env python3
"""
JoyCon 上层逻辑诊断 v2
创建 JoyconRobotics 实例，等待检测到摇杆推动后再开始监控 position 变化。
"""

import sys
import os
import time
import logging

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "joyconrobotics"))
logging.basicConfig(level=logging.DEBUG, format="%(levelname)s: %(message)s")

from joyconrobotics.joyconrobotics import JoyconRobotics

print("=" * 60)
print("  JoyconRobotics 上层诊断 v2")
print("=" * 60)

print("\n[1] 创建 JoyconRobotics (跳过校准)...")
try:
    jr = JoyconRobotics(device="right", without_rest_init=True)
    print("  OK")
except Exception as e:
    print(f"  FAIL: {e}")
    sys.exit(1)

time.sleep(0.5)

print(f"\n[2] 参数检查:")
print(f"  dof_speed    = {jr.dof_speed}")
print(f"  glimit       = {jr.glimit}")
print(f"  if_limit_dof = {jr.if_limit_dof}")
print(f"  position     = {jr.position}")
print(f"  thread alive = {jr.thread.is_alive()}")

is_right = jr.joycon.is_right()

# 等待摇杆被推动
print(f"\n[3] 等待摇杆推动... (现在请前推摇杆! 30秒超时)")
wait_start = time.time()
detected = False
while time.time() - wait_start < 30:
    sv = jr.joycon.get_stick_right_vertical() if is_right else jr.joycon.get_stick_left_vertical()
    if sv > 3000 or sv < 1200:
        print(f"  检测到摇杆推动! stick_v={sv}")
        detected = True
        break
    time.sleep(0.02)

if not detected:
    print("  超时! 30秒内未检测到摇杆推动。")
    print("  stick_v 一直在中心附近，请确认你确实在推动摇杆。")
    jr.running = False
    sys.exit(1)

# 记录推动前后的 position
print(f"\n[4] 监控 position 变化 (3秒, 请持续推动摇杆)...")
initial_pos = jr.position.copy()
print(f"  初始 position = [{initial_pos[0]:+.5f}, {initial_pos[1]:+.5f}, {initial_pos[2]:+.5f}]")

for i in range(15):
    sv = jr.joycon.get_stick_right_vertical() if is_right else jr.joycon.get_stick_left_vertical()
    sh = jr.joycon.get_stick_right_horizontal() if is_right else jr.joycon.get_stick_left_horizontal()
    pos = jr.position.copy()
    dx = pos[0] - initial_pos[0]
    dy = pos[1] - initial_pos[1]

    tag = ""
    if sv > 3750: tag = " [>3750 FWD!]"
    elif sv > 3000: tag = " [>3000]"
    elif sv < 1000: tag = " [<1000 BWD!]"
    elif sv < 1200: tag = " [<1200]"

    print(
        f"  V={sv:5d}{tag:14s}  H={sh:5d}  "
        f"pos=[{pos[0]:+.5f},{pos[1]:+.5f},{pos[2]:+.5f}]  "
        f"dx={dx:+.6f} dy={dy:+.6f}  thr={'alive' if jr.thread.is_alive() else 'DEAD'}"
    )
    time.sleep(0.2)

final_pos = jr.position.copy()
total_dx = final_pos[0] - initial_pos[0]
total_dy = final_pos[1] - initial_pos[1]

print(f"\n[5] 结论:")
print(f"  position 变化: dx={total_dx:+.6f}, dy={total_dy:+.6f}")

if abs(total_dx) > 0.0005 or abs(total_dy) > 0.0005:
    print("  OK: position 有明显变化，JoyconRobotics 内部工作正常!")
    print("  如果 xarm 仍然不动，问题在 IK 求解或 xarm API 调用侧。")
else:
    print("  FAIL: position 几乎没变化!")
    print("  进一步检查:")
    print(f"    glimit x 范围: [{jr.glimit[0][0]}, {jr.glimit[1][0]}]")
    print(f"    当前 position[0]: {final_pos[0]}")
    if jr.if_limit_dof:
        if final_pos[0] >= jr.glimit[1][0] - 0.001:
            print("    *** position[0] 触及 glimit 上限! ***")
        elif final_pos[0] <= jr.glimit[0][0] + 0.001:
            print("    *** position[0] 触及 glimit 下限! ***")

jr.running = False
print("\n诊断完成。")
