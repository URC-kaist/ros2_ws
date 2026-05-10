#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'USAGE'
Usage:
  calibrate_all_nofw_motors.bash [can_iface] [per_node_wait_sec]

Runs NoFW FOC calibration for deployed rover nodes 1..8, one node at a time.
Each node is disarmed before calibration and checked through runtime diagnostic.

Defaults:
  can_iface          = can0
  per_node_wait_sec = 20

Examples:
  ./scripts/calibrate_all_nofw_motors.bash
  ./scripts/calibrate_all_nofw_motors.bash can0 30
USAGE
}

if [[ $# -gt 2 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 2
fi

if ! command -v cansend >/dev/null 2>&1; then
  echo "cansend not found. Install can-utils." >&2
  exit 1
fi

if ! command -v candump >/dev/null 2>&1; then
  echo "candump not found. Install can-utils." >&2
  exit 1
fi

iface="${1:-can0}"
per_node_wait_sec="${2:-20}"
nodes=(1 2 3 4 5 6 7 8 9)

if ! ip link show "${iface}" >/dev/null 2>&1; then
  echo "CAN interface '${iface}' not found or not accessible." >&2
  exit 1
fi

frame_id() {
  printf '%03X' "$1"
}

send_power() {
  local node_id="$1"
  local value="$2"
  cansend "${iface}" "$(frame_id $((0x230 + node_id)))#${value}"
}

send_foc_calibration() {
  local node_id="$1"
  cansend "${iface}" "$(frame_id $((0x290 + node_id)))#01"
}

wait_for_diag_state() {
  local node_id="$1"
  local timeout_sec="$2"
  local diag_id
  diag_id="$(frame_id $((0x5F0 + node_id)))"

  python3 - "${iface}" "${diag_id}" "${timeout_sec}" <<'PY'
import re
import subprocess
import sys
import time

iface, diag_id, timeout_s = sys.argv[1], sys.argv[2].upper(), float(sys.argv[3])
deadline = time.monotonic() + timeout_s
cmd = ["candump", "-L", f"{iface},{diag_id}:7FF"]
proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

try:
    while time.monotonic() < deadline:
        line = proc.stdout.readline()
        if not line:
            time.sleep(0.02)
            continue
        match = re.search(r"([0-9A-Fa-f]{3})#([0-9A-Fa-f]+)", line)
        if not match or match.group(1).upper() != diag_id:
            continue
        payload = bytes.fromhex(match.group(2))
        if len(payload) != 8:
            continue
        foc_valid = bool(payload[4] & 0x04)
        fault = payload[5]
        armed = bool(payload[7] & 0x02)
        need_cal = bool(payload[6] & 0x01)
        print(
            f"{line.strip()} foc_valid={int(foc_valid)} fault={fault} "
            f"armed={int(armed)} need_calibration={int(need_cal)}",
            flush=True,
        )
        if foc_valid and fault == 0 and not armed:
            raise SystemExit(0)
finally:
    proc.terminate()
    try:
        proc.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        proc.kill()

raise SystemExit(1)
PY
}

echo "NoFW FOC calibration on ${iface}"
echo "Nodes: ${nodes[*]}"
echo "Per-node wait: ${per_node_wait_sec}s"
echo
echo "Keep wheels clear. Calibration temporarily enables each motor power stage."
echo

failures=()

for node_id in "${nodes[@]}"; do
  echo "== Node ${node_id} =="
  echo "Disarm"
  send_power "${node_id}" "00"
  sleep 0.2

  echo "Run FOC calibration"
  send_foc_calibration "${node_id}"

  if wait_for_diag_state "${node_id}" "${per_node_wait_sec}"; then
    echo "Node ${node_id}: FOC calibration valid, fault clear, disarmed"
  else
    echo "Node ${node_id}: calibration did not report ready within ${per_node_wait_sec}s" >&2
    failures+=("${node_id}")
  fi

  echo "Disarm"
  send_power "${node_id}" "00"
  sleep 0.5
  echo
done

if (( ${#failures[@]} > 0 )); then
  echo "Failed or timed out nodes: ${failures[*]}" >&2
  exit 1
fi

echo "All nodes reported trusted FOC calibration valid, no runtime fault, disarmed."
