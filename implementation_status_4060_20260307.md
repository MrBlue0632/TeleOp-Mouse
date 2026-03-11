# 4060 键鼠采集实现状态（2026-03-07）

已在 4060 端实现并通过自动检查。

## 4060 已修改文件

- `/home/bozhao_4060_2/ros_ws/src/LeRobot-Anything-U-Arm-main/src/uarm/scripts/keyboard_mouse_teleop.py`（新增）
- `/home/bozhao_4060_2/ros_ws/src/LeRobot-Anything-U-Arm-main/src/uarm/scripts/episode_recorder.py`（修改：支持 Enter 结束当前 episode）
- `/home/bozhao_4060_2/ros_ws/src/LeRobot-Anything-U-Arm-main/src/uarm/scripts/run_all_nodes.sh`（修改：简化为 cam/xarm/teleop/recorder）
- `/home/bozhao_4060_2/ros_ws/src/LeRobot-Anything-U-Arm-main/src/uarm/scripts/check_keyboard_mouse_teleop.sh`（新增：一键自检）

## 控制映射

- `W/S/A/D`: `x+/x-/y+/y-`
- `Shift/Space`: `z-/z+`
- 鼠标移动: 上下 -> `RY +/-`，左右 -> `RX -/+`
- 鼠标左键: 夹爪渐变关闭
- 鼠标右键: 夹爪渐变打开
- `R`: home
- `Enter`: 结束当前 episode（recorder 保存并切到下一段）

## 速度默认值（慢速）

- `linear_step_mm=0.6`
- `angular_step_deg_per_px=0.06`
- `cart_speed=20.0`
- `gripper_step=10.0`

## 自动检查（已执行通过）

在 4060 上执行：

```bash
~/ros_ws/src/LeRobot-Anything-U-Arm-main/src/uarm/scripts/check_keyboard_mouse_teleop.sh
```

输出包含：`[CHECK] PASS`

## 启动方式

在 4060 上执行：

```bash
cd ~/ros_ws/src/LeRobot-Anything-U-Arm-main/src/uarm/scripts
./run_all_nodes.sh
```

