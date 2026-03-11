#!/bin/bash

# 根据脚本位置自动定位工作空间
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$( cd "$SCRIPT_DIR/../../../../" && pwd )"

# 捕获 Ctrl+C 信号并停止所有后台进程
trap "kill $PID1 $PID2 $PID3 2>/dev/null" SIGINT

# === 设置 ROS 环境变量 ===
echo "Setting up ROS environment (workspace: $WORKSPACE_DIR)..."
source "$WORKSPACE_DIR/devel/setup.bash"

# === 使用绝对路径启动脚本，避免 rosrun 解析到旧工作空间 ===

echo "Starting lerobot_joycon_gpos_xarm6_new.py..."
python3 "$SCRIPT_DIR/examples/lerobot_joycon_gpos_xarm6_new.py" &
PID1=$!
echo "[INFO] lerobot_joycon_gpos_xarm6_new.py started with PID $PID1"
sleep 2

echo "Starting cam_pub.py..."
python3 "$SCRIPT_DIR/cam_pub.py" &
PID2=$!
echo "[INFO] cam_pub.py started with PID $PID2"
sleep 2

echo "Starting episode_recorder.py..."
python3 "$SCRIPT_DIR/episode_recorder.py" &
PID3=$!
echo "[INFO] episode_recorder.py started with PID $PID3"

# 等待所有后台进程完成
wait $PID1
wait $PID2
wait $PID3

echo "[INFO] All nodes finished."
