#!/bin/bash
set -euo pipefail

ROOT_DIR="/home/zyl5090/Projects/Teleop"
LOG_DIR="${ROOT_DIR}/logs"
mkdir -p "${LOG_DIR}"
TS="$(date +%Y%m%d_%H%M%S)"
TELEOP_LOG="${LOG_DIR}/teleop_${TS}.log"
MON_LOG="${LOG_DIR}/monitor_${TS}.csv"

cleanup() {
  if [[ -n "${MON_PID:-}" ]] && kill -0 "${MON_PID}" 2>/dev/null; then
    kill "${MON_PID}" 2>/dev/null || true
    wait "${MON_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

"${ROOT_DIR}/scripts/teleop_runtime_monitor.sh" "${MON_LOG}" &
MON_PID=$!

echo "[MON] runtime monitor pid=${MON_PID}, log=${MON_LOG}"
echo "[RUN] teleop log -> ${TELEOP_LOG}"

# Keep teleop in foreground for keyboard/mouse capture.
set +e
stdbuf -oL -eL "${ROOT_DIR}/scripts/run_5090_teleop.sh" "$@" 2>&1 | tee "${TELEOP_LOG}"
RET=${PIPESTATUS[0]}
set -e

echo "[DONE] teleop exit=${RET}"
echo "[DONE] monitor log=${MON_LOG}"
exit "${RET}"
