#!/bin/bash
set -euo pipefail

OUT_FILE="${1:-}"
if [[ -z "${OUT_FILE}" ]]; then
  echo "usage: $0 <out_file>" >&2
  exit 1
fi

mkdir -p "$(dirname "${OUT_FILE}")"

echo "ts,pid,cpu_pct,mem_pct,rss_mb,vsz_mb,load1,mem_avail_mb,gpu_util_pct,gpu_mem_util_pct,gpu_mem_used_mb,gpu_mem_total_mb,gpu_temp_c,tunnel_tcp_count,tunnel_retrans,ssh_tunnel_cpu_pct,ssh_tunnel_mem_pct" > "${OUT_FILE}"

find_pid() {
  ps -eo pid=,args= 2>/dev/null | awk '
    $0 ~ /python3 .*\/scripts\/teleop_5090_keyboard_mouse\.py/ {print $1}
  ' | tail -n1
}

while true; do
  pid="$(find_pid)"
  if [[ -n "${pid}" ]]; then
    break
  fi
  sleep 0.2
done

while kill -0 "${pid}" 2>/dev/null; do
  ts="$(date '+%Y-%m-%d %H:%M:%S')"

  ps_line="$(ps -p "${pid}" -o %cpu=,%mem=,rss=,vsz= 2>/dev/null | awk '{print $1","$2","$3","$4}')"
  if [[ -z "${ps_line}" ]]; then
    ps_line=",,,"
  fi
  cpu_pct="$(echo "${ps_line}" | cut -d, -f1)"
  mem_pct="$(echo "${ps_line}" | cut -d, -f2)"
  rss_kb="$(echo "${ps_line}" | cut -d, -f3)"
  vsz_kb="$(echo "${ps_line}" | cut -d, -f4)"

  rss_mb="0"
  vsz_mb="0"
  if [[ -n "${rss_kb}" ]]; then
    rss_mb="$(awk -v k="${rss_kb}" 'BEGIN{printf "%.1f", k/1024.0}')"
  fi
  if [[ -n "${vsz_kb}" ]]; then
    vsz_mb="$(awk -v k="${vsz_kb}" 'BEGIN{printf "%.1f", k/1024.0}')"
  fi

  load1="$(awk '{print $1}' /proc/loadavg 2>/dev/null || echo "")"
  mem_avail_kb="$(awk '/MemAvailable:/ {print $2}' /proc/meminfo 2>/dev/null || echo "0")"
  mem_avail_mb="$(awk -v k="${mem_avail_kb}" 'BEGIN{printf "%.1f", k/1024.0}')"

  gpu_util=""
  gpu_mem_util=""
  gpu_mem_used=""
  gpu_mem_total=""
  gpu_temp=""
  if command -v nvidia-smi >/dev/null 2>&1; then
    gpu_line="$(nvidia-smi --query-gpu=utilization.gpu,utilization.memory,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits 2>/dev/null | head -n1 || true)"
    if [[ -n "${gpu_line}" ]]; then
      gpu_util="$(echo "${gpu_line}" | awk -F',' '{gsub(/ /, "", $1); print $1}')"
      gpu_mem_util="$(echo "${gpu_line}" | awk -F',' '{gsub(/ /, "", $2); print $2}')"
      gpu_mem_used="$(echo "${gpu_line}" | awk -F',' '{gsub(/ /, "", $3); print $3}')"
      gpu_mem_total="$(echo "${gpu_line}" | awk -F',' '{gsub(/ /, "", $4); print $4}')"
      gpu_temp="$(echo "${gpu_line}" | awk -F',' '{gsub(/ /, "", $5); print $5}')"
    fi
  fi

  ss_dump="$(ss -tin '( sport = :1502 or sport = :13001 or sport = :13002 or sport = :13003 or sport = :18333 or dport = :1502 or dport = :13001 or dport = :13002 or dport = :13003 or dport = :18333 )' 2>/dev/null || true)"
  tunnel_tcp_count="$(echo "${ss_dump}" | awk '/ESTAB/ {c+=1} END{print c+0}')"
  tunnel_retrans="$( (echo "${ss_dump}" | grep -o 'retrans:[0-9]\+/[0-9]\+' | awk -F'[:/]' '{s+=$2} END{print s+0}') || true )"
  if [[ -z "${tunnel_retrans}" ]]; then
    tunnel_retrans="0"
  fi

  ssh_ps="$(ps -C ssh -o %cpu=,%mem=,args= 2>/dev/null | grep -E -- '-L (1502|13001|13002|13003|18333):' || true)"
  if [[ -n "${ssh_ps}" ]]; then
    ssh_tunnel_cpu="$(echo "${ssh_ps}" | awk '{s+=$1} END{printf "%.1f", s+0}')"
    ssh_tunnel_mem="$(echo "${ssh_ps}" | awk '{s+=$2} END{printf "%.1f", s+0}')"
  else
    ssh_tunnel_cpu="0.0"
    ssh_tunnel_mem="0.0"
  fi

  echo "${ts},${pid},${cpu_pct},${mem_pct},${rss_mb},${vsz_mb},${load1},${mem_avail_mb},${gpu_util},${gpu_mem_util},${gpu_mem_used},${gpu_mem_total},${gpu_temp},${tunnel_tcp_count},${tunnel_retrans},${ssh_tunnel_cpu},${ssh_tunnel_mem}" >> "${OUT_FILE}"
  sleep 1
done
