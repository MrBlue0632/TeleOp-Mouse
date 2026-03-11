# 5090 本地运行说明（无 ROS）

适用场景：4060 无图形界面，键鼠操作与实时电流显示都在 5090 完成。

## 启动

```bash
cd /home/zyl5090/Projects/Teleop
./scripts/run_5090_teleop.sh
```

该脚本会自动：
- 检查并建立 5090 -> 4060 -> 机械臂的 SSH 隧道（本地 `1502 -> 502`，以及本地 `13001/13002/13003 -> 30001/30002/30003`），并额外映射 `192.168.1.199:18333` 到本机
- 检查 Python 依赖
- 启动本地键鼠 teleop
- 每次启动先按 4060 参考位姿复位（关节角：`[14.1, -8, -24.7, 196.9, 62.3, -8.8]`）

## 视频与电流面板（独立于终端输出）

- 默认优先显示 `/dev/video4` 实时画面。
- 若首选节点无帧，会自动回退到其他可用相机源。
- 画面右侧有独立的电流面板（滤波显示），与终端原始电流输出是两条独立链路。
- `GRIP` 电流优先采用 4060 `monitor_gripper_current.py` 同源寄存器读取（`0x0003`），并带 `CAL:OK/WARN` 校对状态。
- 若需要禁用视频：

```bash
python3 scripts/teleop_5090_keyboard_mouse.py --robot-ip 127.0.0.1 --no-video
```

## FPS 鼠标模式与退出

- 默认启用 FPS 鼠标模式：鼠标固定回屏幕中心（用于低延迟方向控制）。
- 系统有 `unclutter` 时会自动隐藏鼠标指针。
- `ESC` 可随时退出。
- 若要关闭 FPS 鼠标模式：

```bash
./scripts/run_5090_teleop.sh --no-fps-mouse
```

## 键鼠映射

- `W/S/A/D`：`x+/x-/y+/y-`
- `Shift/Space`：`z-/z+`
- 鼠标左右：`RX -/+`
- 鼠标上下：`RY +/-`
- 鼠标左键：夹爪渐变关闭
- 鼠标右键：夹爪渐变打开
- `R`：回 home
- `Enter`：结束当前 episode 并保存

## 实时电流显示

运行时终端会实时刷新：
- `J1..J6`：六个关节电流
- `GRIP`：夹爪电流（优先寄存器 `0x0003`，失败时回退 SDK 第7轴）

## 响应与速度

- 速度已调整为上一版约 `2x`。
- 控制环以“仅发送当前最新目标”的方式运行，减少历史动作堆积带来的延迟感。

## 数据保存

- 保存目录：`/home/zyl5090/Projects/Teleop/data`
- 格式：`episode_XXXX_YYYYmmdd_HHMMSS.jsonl`

## 自检

```bash
python3 scripts/teleop_5090_keyboard_mouse.py --self-check
python3 -m py_compile scripts/teleop_5090_keyboard_mouse.py
```
