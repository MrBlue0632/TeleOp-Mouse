#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_PATH="${ROOT_DIR}/scripts/teleop_keyboard_mouse.py"
DATA_DIR="${ROOT_DIR}/data"

cleanup_local_teleop() {
  local pids
  pids="$(pgrep -f "${SCRIPT_PATH}" || true)"
  if [[ -n "${pids}" ]]; then
    echo "[INFO] stopping stale TeleOp_clean processes: ${pids}"
    kill ${pids} 2>/dev/null || true
    sleep 0.5
    kill -9 ${pids} 2>/dev/null || true
  fi
}

ensure_python_deps() {
  python3 - <<'PY'
import importlib
missing = []
for module in ("pynput", "xarm", "numpy", "cv2"):
    try:
        importlib.import_module(module)
    except Exception:
        missing.append(module)
if missing:
    print("MISSING:" + ",".join(missing))
    raise SystemExit(1)
print("PY_DEPS_OK")
PY
}

show_help() {
  cat <<'EOF'
Usage: ./scripts/run_local_teleop.sh [teleop args...]

Local-only launcher. No SSH tunnel logic is included.

Common overrides:
  --robot-ip 192.168.1.199
  --camera-dev /dev/video6
  --camera-id 1
  --allow-camera-fallback
  --no-video
  --no-fps-mouse

Examples:
  ./scripts/run_local_teleop.sh
  ./scripts/run_local_teleop.sh --camera-dev /dev/video6
  ./scripts/run_local_teleop.sh --robot-ip 192.168.1.199 --no-fps-mouse
EOF
}

for arg in "$@"; do
  case "$arg" in
    -h|--help)
      show_help
      exit 0
      ;;
  esac
done

cleanup_local_teleop
ensure_python_deps
mkdir -p "${DATA_DIR}"

exec python3 "${SCRIPT_PATH}" \
  --robot-ip "${ROBOT_IP:-192.168.1.199}" \
  --robot-port 502 \
  --report-port-normal 30001 \
  --report-port-rich 30002 \
  --report-port-real 30003 \
  --rate-hz 30 \
  --control-hz 120 \
  --camera-id 4 \
  --camera-dev /dev/video4 \
  --allow-camera-fallback \
  --data-dir "${DATA_DIR}" \
  "$@"
