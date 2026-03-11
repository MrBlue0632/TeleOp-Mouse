#!/bin/bash
set -euo pipefail

REMOTE_USER_HOST="bozhao_4060_2@192.168.31.249"

cleanup_local_teleop() {
  local pids
  pids="$(pgrep -f '/home/zyl5090/Projects/Teleop/scripts/teleop_5090_keyboard_mouse.py' || true)"
  if [[ -n "${pids}" ]]; then
    echo "[INFO] stopping stale local teleop processes: ${pids}"
    kill ${pids} 2>/dev/null || true
    sleep 0.5
    kill -9 ${pids} 2>/dev/null || true
  fi
}

# xArm SDK on local 5090 needs local high ports mapped to robot controller via 4060.
# Also expose 18333 locally for the service running on the robot-side network.
ensure_tunnel() {
  for p in 1502 13001 13002 13003 18333; do
    if ! timeout 1 bash -lc "cat < /dev/null > /dev/tcp/127.0.0.1/${p}" >/dev/null 2>&1; then
      echo "[INFO] opening SSH tunnel for port ${p}"
      ssh -o BatchMode=yes -fN -o ExitOnForwardFailure=yes \
        -L 1502:192.168.1.199:502 \
        -L 13001:192.168.1.199:30001 \
        -L 13002:192.168.1.199:30002 \
        -L 13003:192.168.1.199:30003 \
        -L 18333:192.168.1.199:18333 \
        "${REMOTE_USER_HOST}"
      break
    fi
  done

  for p in 1502 13001 13002 13003 18333; do
    timeout 2 bash -lc "cat < /dev/null > /dev/tcp/127.0.0.1/${p}" || {
      echo "[ERROR] tunnel check failed at ${p}"
      exit 1
    }
  done
}

ensure_python_deps() {
  python3 - <<'PY'
import importlib
missing=[]
for m in ['pynput','xarm','numpy','cv2']:
    try:
        importlib.import_module(m)
    except Exception:
        missing.append(m)
if missing:
    print('MISSING:' + ','.join(missing))
    raise SystemExit(1)
print('PY_DEPS_OK')
PY
}

show_help() {
  cat <<'EOF'
Usage: ./scripts/run_5090_teleop.sh [teleop args...]

Common camera overrides:
  --camera-id N          Use integer camera index N
  --camera-dev /dev/...  Use explicit device node
  --allow-camera-fallback
  --no-video

Examples:
  ./scripts/run_5090_teleop.sh --camera-id 1
  ./scripts/run_5090_teleop.sh --camera-dev /dev/video6
  ./scripts/run_5090_teleop.sh --camera-dev /dev/video4 --no-fps-mouse
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
ensure_tunnel
ensure_python_deps

exec python3 /home/zyl5090/Projects/Teleop/scripts/teleop_5090_keyboard_mouse.py \
  --robot-ip 127.0.0.1 \
  --robot-port 1502 \
  --rate-hz 30 \
  --control-hz 120 \
  --camera-id 4 \
  --camera-dev /dev/video4 \
  --allow-camera-fallback \
  --data-dir /home/zyl5090/Projects/Teleop/data \
  "$@"
