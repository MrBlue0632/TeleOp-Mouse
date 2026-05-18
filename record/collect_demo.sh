#!/bin/bash
set -euo pipefail

# ============================================================
#  collect_demo.sh — 采集 demo 数据集
#
#  用法:
#    ./record/collect_demo.sh                          # 默认参数
#    ./record/collect_demo.sh --task "pick_red_cube"    # 自定义任务描述
#    ./record/collect_demo.sh --no-video                # 不显示视频窗口
#    ./record/collect_demo.sh --display-camera target   # 显示 target 相机
# ============================================================

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

# ---------- 默认配置 ----------
ROBOT_IP="${ROBOT_IP:-192.168.1.199}"
DATA_DIR="${DATA_DIR:-${ROOT_DIR}/data}"
REPO_ID="${REPO_ID:-teleop/xarm_demo}"
TASK="${TASK:-keyboard_mouse_teleop}"
RATE_HZ="${RATE_HZ:-30}"
VCODEC="${VCODEC:-libsvtav1}"
WEB_DASHBOARD="${WEB_DASHBOARD:-0}"
WEB_HOST="${WEB_HOST:-127.0.0.1}"
WEB_PORT="${WEB_PORT:-8765}"
WEB_FPS="${WEB_FPS:-10}"

# ---------- 相机配置 ----------
WRIST_CAM_ID="${WRIST_CAM_ID:-10}"
WRIST_CAM_DEV="${WRIST_CAM_DEV:-/dev/video10}"
BASE_CAM_ID="${BASE_CAM_ID:-16}"
BASE_CAM_DEV="${BASE_CAM_DEV:-/dev/video16}"
TARGET_CAM_ID="${TARGET_CAM_ID:-4}"
TARGET_CAM_DEV="${TARGET_CAM_DEV:-/dev/video4}"
DISPLAY_CAMERA="${DISPLAY_CAMERA:-wrist}"

# ---------- X display (needed for pynput via SSH) ----------
export DISPLAY="${DISPLAY:-:0}"

# ---------- conda 环境 ----------
if [ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]; then
    source "${HOME}/anaconda3/etc/profile.d/conda.sh"
    if [ -n "${TELEOP_CONDA_ENV:-}" ]; then
        conda activate "${TELEOP_CONDA_ENV}"
    elif [ -z "${CONDA_DEFAULT_ENV:-}" ]; then
        conda activate xarm
    fi
fi

# lerobot 源码路径（如果未通过 pip 安装）
if [ -d "${HOME}/lerobot/src" ]; then
    export PYTHONPATH="${HOME}/lerobot/src:${PYTHONPATH:-}"
fi

# ---------- 依赖检查 ----------
python3 - <<'PY' || exit 1
import importlib.util
import sys
required = ["pynput", "xarm", "numpy", "cv2", "lerobot", "pybullet"]
missing = []
for m in required:
    if importlib.util.find_spec(m) is None:
        missing.append(m)
if missing:
    print(f"[ERROR] missing packages: {', '.join(missing)}", file=sys.stderr)
    sys.exit(1)
print("[OK] all dependencies available")
PY

# ---------- 清理旧进程 ----------
OLD_PIDS="$(pgrep -f "main.py|record.main|record/teleop.py" || true)"
if [ -n "${OLD_PIDS}" ]; then
    echo "[INFO] stopping stale teleop processes: ${OLD_PIDS}"
    kill ${OLD_PIDS} 2>/dev/null || true
    sleep 0.5
fi

# ---------- 解析参数 ----------
EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --task)            TASK="$2"; shift 2 ;;
        --repo-id)         REPO_ID="$2"; shift 2 ;;
        --data-dir)        DATA_DIR="$2"; shift 2 ;;
        --rate-hz)         RATE_HZ="$2"; shift 2 ;;
        --vcodec)          VCODEC="$2"; shift 2 ;;
        --wrist-cam-dev)   WRIST_CAM_DEV="$2"; shift 2 ;;
        --base-cam-dev)    BASE_CAM_DEV="$2"; shift 2 ;;
        --target-cam-dev)  TARGET_CAM_DEV="$2"; shift 2 ;;
        --display-camera)  DISPLAY_CAMERA="$2"; shift 2 ;;
        --web-dashboard)   WEB_DASHBOARD="1"; shift ;;
        --web-host)        WEB_HOST="$2"; shift 2 ;;
        --web-port)        WEB_PORT="$2"; shift 2 ;;
        --web-fps)         WEB_FPS="$2"; shift 2 ;;
        *)                 EXTRA_ARGS+=("$1"); shift ;;
    esac
done

WEB_ARGS=(--web-host "${WEB_HOST}" --web-port "${WEB_PORT}" --web-fps "${WEB_FPS}")
if [[ "${WEB_DASHBOARD}" == "1" || "${WEB_DASHBOARD}" == "true" || "${WEB_DASHBOARD}" == "TRUE" ]]; then
    WEB_ARGS=(--web-dashboard "${WEB_ARGS[@]}")
fi

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
echo "  Robot IP      : ${ROBOT_IP}:${ROBOT_PORT}"
echo "  Data Dir      : ${DATA_DIR}"
echo "  Repo ID       : ${REPO_ID}"
echo "  Task          : ${TASK}"
echo "  FPS           : ${RATE_HZ}"
echo "  Codec         : ${VCODEC}"
echo "  Web Dashboard : ${WEB_DASHBOARD} (${WEB_HOST}:${WEB_PORT}, ${WEB_FPS} fps)"
echo "  Torque Data   : raw/model/external/bias + EE wrench"
echo "  ---"
echo "  Wrist Camera  : ${WRIST_CAM_DEV}"
echo "  Base Camera   : ${BASE_CAM_DEV}"
echo "  Target Camera : ${TARGET_CAM_DEV}"
echo "  Display       : ${DISPLAY_CAMERA}"
echo "============================================"
echo ""
echo "  Controls:"
echo "    WASD/Space/Shift  — move end-effector"
echo "    Q/E               — rotate Z-axis"
echo "    Mouse move        — rotate X/Y-axis"
echo "    Left click        — toggle gripper"
echo "    Tab               — toggle BASE/TOOL frame"
echo "    1/2/3             — speed 0.5x/1x/2x"
echo "    R                 — return to home"
echo "    Enter             — save episode"
echo "    ESC / Ctrl+C      — exit"
echo "============================================"
echo ""

# ---------- 启动遥操作 ----------
exec python3 "${ROOT_DIR}/main.py" \
    --robot-ip "${ROBOT_IP}" \
    --robot-port "${ROBOT_PORT}" \
    --report-port-normal "${REPORT_NORM}" \
    --report-port-rich "${REPORT_RICH}" \
    --report-port-real "${REPORT_REAL}" \
    --rate-hz "${RATE_HZ}" \
    --control-hz 120 \
    --camera-dev "${WRIST_CAM_DEV}" \
    --base-camera-dev "${BASE_CAM_DEV}" \
    --target-camera-dev "${TARGET_CAM_DEV}" \
    --display-camera "${DISPLAY_CAMERA}" \
    --allow-camera-fallback \
    --data-dir "${DATA_DIR}" \
    --repo-id "${REPO_ID}" \
    --task "${TASK}" \
    --vcodec "${VCODEC}" \
    "${WEB_ARGS[@]}" \
    "${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}"
