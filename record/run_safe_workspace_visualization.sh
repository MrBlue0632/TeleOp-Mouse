#!/bin/bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

show_help() {
  cat <<'EOF'
Usage: ./record/run_safe_workspace_visualization.sh [options]

Runs a conservative automatic xArm safe-workspace motion loop while publishing:
  - external joint torque observation
  - EEF-frame force arrow telemetry
  - live 3D URDF dashboard visualization

Common options:
  --web-host 127.0.0.1
  --web-port 8765
  --amplitude-deg 8,6,6,10,8,8
  --speed-deg-s 8
  --cycles 0                 # 0 repeats until Ctrl+C
  --record-dataset           # optional; default is telemetry-only
  --self-check               # validate config without connecting to the robot

Environment:
  TELEOP_CONDA_ENV=base      # conda env to activate; auto-selects xarm then base
  ROBOT_IP=192.168.1.199     # override robot IP
  STOP_STALE_TELEOP=1        # stop stale teleop processes before launch
EOF
}

for arg in "$@"; do
  case "${arg}" in
    -h|--help)
      show_help
      exit 0
      ;;
  esac
done

export DISPLAY="${DISPLAY:-:0}"
export PYTHONUNBUFFERED=1

if [ -f "${HOME}/anaconda3/etc/profile.d/conda.sh" ]; then
  source "${HOME}/anaconda3/etc/profile.d/conda.sh"
  if [ -n "${TELEOP_CONDA_ENV:-}" ]; then
    conda activate "${TELEOP_CONDA_ENV}"
  elif conda env list | awk '{print $1}' | grep -qx 'xarm'; then
    conda activate xarm
  else
    conda activate base
  fi
fi

if [ -d "${HOME}/lerobot/src" ]; then
  export PYTHONPATH="${HOME}/lerobot/src:${PYTHONPATH:-}"
fi

DEFAULT_TORQUE_COMP_MODEL="${ROOT_DIR}/dynamics/calibration/compensation/history_q_qd.pt"
if [ -z "${TELEOP_TORQUE_COMP_MODEL:-}" ] && [ -f "${DEFAULT_TORQUE_COMP_MODEL}" ]; then
  export TELEOP_TORQUE_COMP_MODEL="${DEFAULT_TORQUE_COMP_MODEL}"
fi

python3 - <<'PYDEP'
import importlib.util
import sys
required = ["numpy", "yaml", "cv2", "lerobot", "xarm"]
missing = [name for name in required if importlib.util.find_spec(name) is None]
if missing:
    print("[ERROR] missing packages: " + ", ".join(missing), file=sys.stderr)
    raise SystemExit(1)
print("[OK] runtime dependencies available")
PYDEP

OLD_PIDS="$(pgrep -f 'record/teleop.py|record.main|safe_workspace_visualization.py|admittance_control.py' | grep -v "^$$$" || true)"
if [ -n "${OLD_PIDS}" ]; then
  if [[ "${STOP_STALE_TELEOP:-0}" == "1" ]]; then
    echo "[INFO] stopping stale teleop/admittance processes: ${OLD_PIDS}"
    kill -INT ${OLD_PIDS} 2>/dev/null || true
    sleep 1.0
  else
    echo "[ERROR] stale teleop/admittance process detected: ${OLD_PIDS}" >&2
    echo "        Re-run with STOP_STALE_TELEOP=1 to stop it before launch." >&2
    exit 1
  fi
fi

mkdir -p "${ROOT_DIR}/data"

echo "[INFO] launching safe workspace visualization runner"
exec python3 "${ROOT_DIR}/record/safe_workspace_visualization.py" "$@"
