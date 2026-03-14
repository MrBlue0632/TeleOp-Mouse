#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_PATH="${ROOT_DIR}/scripts/teleop_keyboard_mouse.py"
DATA_DIR="${ROOT_DIR}/data"

has_local_forward() {
  python3 - <<'PY'
import socket
ports = (1502, 13001, 13002, 13003)
for port in ports:
    s = socket.socket()
    s.settimeout(0.3)
    try:
        s.connect(("127.0.0.1", port))
    except Exception:
        raise SystemExit(1)
    finally:
        s.close()
print("LOCAL_FORWARD_OK")
PY
}

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

ROBOT_IP_VALUE="${ROBOT_IP:-192.168.1.199}"
ROBOT_PORT_VALUE=502
REPORT_NORM_VALUE=30001
REPORT_RICH_VALUE=30002
REPORT_REAL_VALUE=30003

if has_local_forward >/dev/null 2>&1; then
  echo "[INFO] using existing local forwarded xArm ports on 127.0.0.1"
  ROBOT_IP_VALUE="127.0.0.1"
  ROBOT_PORT_VALUE=1502
  REPORT_NORM_VALUE=13001
  REPORT_RICH_VALUE=13002
  REPORT_REAL_VALUE=13003
else
  echo "[INFO] using direct xArm connection at ${ROBOT_IP_VALUE}"
fi

exec python3 "${SCRIPT_PATH}" \
  --robot-ip "${ROBOT_IP_VALUE}" \
  --robot-port "${ROBOT_PORT_VALUE}" \
  --report-port-normal "${REPORT_NORM_VALUE}" \
  --report-port-rich "${REPORT_RICH_VALUE}" \
  --report-port-real "${REPORT_REAL_VALUE}" \
  --rate-hz 30 \
  --control-hz 120 \
  --camera-id 1 \
  --allow-camera-fallback \
  --data-dir "${DATA_DIR}" \
  "$@"
