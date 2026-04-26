#!/bin/bash
set -euo pipefail

# ============================================================
#  collect_demo.sh — 采集 demo 数据集
#
#  用法:
#    ./scripts/collect_demo.sh                          # 默认参数
#    ./scripts/collect_demo.sh --task "pick_red_cube"    # 自定义任务描述
#    ./scripts/collect_demo.sh --no-video                # 不显示视频窗口
# ============================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_PATH="${ROOT_DIR}/scripts/teleop_keyboard_mouse.py"

# ---------- 默认配置 ----------
ROBOT_IP="${ROBOT_IP:-192.168.1.199}"
DATA_DIR="${DATA_DIR:-${ROOT_DIR}/data}"
REPO_ID="${REPO_ID:-teleop/xarm_demo}"
TASK="${TASK:-keyboard_mouse_teleop}"
RATE_HZ="${RATE_HZ:-30}"
VCODEC="${VCODEC:-libsvtav1}"
CAMERA_ID="${CAMERA_ID:-1}"

# ---------- conda 环境 ----------
if [ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]; then
    source "${HOME}/anaconda3/etc/profile.d/conda.sh"
    conda activate base
fi

# lerobot 源码路径（如果未通过 pip 安装）
if [ -d "${HOME}/lerobot/src" ]; then
    export PYTHONPATH="${HOME}/lerobot/src:${PYTHONPATH:-}"
fi

# ---------- 依赖检查 ----------
python3 - <<'PY'
import importlib, sys
required = ["pynput", "xarm", "numpy", "cv2", "lerobot"]
missing = []
for m in required:
    try:
        importlib.import_module(m)
    except ImportError:
        missing.append(m)
if missing:
    print(f"[ERROR] missing packages: {', '.join(missing)}", file=sys.stderr)
    sys.exit(1)
print("[OK] all dependencies available")
PY

# ---------- 清理旧进程 ----------
OLD_PIDS="$(pgrep -f "teleop_keyboard_mouse.py" || true)"
if [ -n "${OLD_PIDS}" ]; then
    echo "[INFO] stopping stale teleop processes: ${OLD_PIDS}"
    kill ${OLD_PIDS} 2>/dev/null || true
    sleep 0.5
fi

# ---------- 解析参数（在端口检测前，以便 --task 等生效） ----------
EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --task)       TASK="$2"; shift 2 ;;
        --repo-id)    REPO_ID="$2"; shift 2 ;;
        --data-dir)   DATA_DIR="$2"; shift 2 ;;
        --rate-hz)    RATE_HZ="$2"; shift 2 ;;
        --vcodec)     VCODEC="$2"; shift 2 ;;
        --camera-id)  CAMERA_ID="$2"; shift 2 ;;
        *)            EXTRA_ARGS+=("$1"); shift ;;
    esac
done

# ---------- 端口转发检测 ----------
ROBOT_PORT=502
REPORT_NORM=30001
REPORT_RICH=30002
REPORT_REAL=30003

if python3 -c "
import socket
for p in (1502, 13001, 13002, 13003):
    s = socket.socket(); s.settimeout(0.3)
    try: s.connect(('127.0.0.1', p))
    except: exit(1)
    finally: s.close()
" 2>/dev/null; then
    echo "[INFO] using local forwarded ports on 127.0.0.1"
    ROBOT_IP="127.0.0.1"
    ROBOT_PORT=1502
    REPORT_NORM=13001
    REPORT_RICH=13002
    REPORT_REAL=13003
else
    echo "[INFO] direct connection to xArm at ${ROBOT_IP}"
fi

# ---------- 创建数据目录 ----------
mkdir -p "${DATA_DIR}"

# ---------- 启动信息 ----------
echo "============================================"
echo "  Demo Data Collection"
echo "============================================"
echo "  Robot IP  : ${ROBOT_IP}:${ROBOT_PORT}"
echo "  Data Dir  : ${DATA_DIR}"
echo "  Repo ID   : ${REPO_ID}"
echo "  Task      : ${TASK}"
echo "  FPS       : ${RATE_HZ}"
echo "  Codec     : ${VCODEC}"
echo "  Camera    : ${CAMERA_ID}"
echo "============================================"
echo ""
echo "  Controls:"
echo "    WASD/Space/Shift  — move end-effector"
echo "    Q/E               — rotate Z-axis"
echo "    Mouse move        — rotate X/Y-axis"
echo "    Left click        — toggle gripper"
echo "    1/2/3             — speed 0.5x/1x/2x"
echo "    R                 — return to home"
echo "    Enter             — save episode"
echo "    ESC / Ctrl+C      — exit"
echo "============================================"
echo ""

# ---------- 启动遥操作 ----------
exec python3 "${SCRIPT_PATH}" \
    --robot-ip "${ROBOT_IP}" \
    --robot-port "${ROBOT_PORT}" \
    --report-port-normal "${REPORT_NORM}" \
    --report-port-rich "${REPORT_RICH}" \
    --report-port-real "${REPORT_REAL}" \
    --rate-hz "${RATE_HZ}" \
    --control-hz 120 \
    --camera-id "${CAMERA_ID}" \
    --allow-camera-fallback \
    --data-dir "${DATA_DIR}" \
    --repo-id "${REPO_ID}" \
    --task "${TASK}" \
    --vcodec "${VCODEC}" \
    "${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}"
