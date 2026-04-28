#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-can0}"

# Steering output actuator node IDs from rover_can.ros2_control.xacro:
#   rr=1, fl=2, fr=3, rl=4
NODE_IDS=(1 2 3 4)
RATE_HZ=20
HALF_SWEEP_SEC=3
HALF_ITERATIONS=$((RATE_HZ * HALF_SWEEP_SEC))
HOLD_ZERO_SEC=2
HOLD_ZERO_ITERATIONS=$((RATE_HZ * HOLD_ZERO_SEC))
PERIOD_SEC="0.05"

if ! command -v cansend >/dev/null 2>&1; then
  echo "cansend not found. Install can-utils." >&2
  exit 1
fi

if ! ip link show "${IFACE}" >/dev/null 2>&1; then
  echo "CAN interface '${IFACE}' not found or not accessible." >&2
  exit 1
fi

send_to_nodes() {
  local frame_base="$1"
  local payload="$2"
  local id
  for id in "${NODE_IDS[@]}"; do
    cansend "${IFACE}" "${frame_base}${id}#${payload}"
  done
}

send_sweep() {
  local label="$1"
  local start_mdeg="$2"
  local end_mdeg="$3"
  local i
  local mdeg
  local payload
  local denom=$((HALF_ITERATIONS - 1))

  echo "${label}: ${RATE_HZ} Hz for ${HALF_SWEEP_SEC}s (${HALF_ITERATIONS} cycles)"
  python3 - "${start_mdeg}" "${end_mdeg}" "${HALF_ITERATIONS}" <<'PY' | while read -r payload; do
import struct
import sys

start = int(sys.argv[1])
end = int(sys.argv[2])
count = int(sys.argv[3])
denom = max(1, count - 1)
for i in range(count):
    value = round(start + (end - start) * (i / denom))
    print(struct.pack("<i", int(value)).hex().upper(), flush=True)
PY
    send_to_nodes "20" "${payload}"
    sleep "${PERIOD_SEC}"
  done
  echo "${label}: done"
}

hold_angle() {
  local label="$1"
  local payload="$2"
  local iterations="$3"
  local i

  echo "${label}: ${RATE_HZ} Hz for $((iterations / RATE_HZ))s (${iterations} cycles)"
  for ((i = 0; i < iterations; ++i)); do
    send_to_nodes "20" "${payload}"
    sleep "${PERIOD_SEC}"
  done
  echo "${label}: done"
}

disarm() {
  send_to_nodes "23" "00"
}

trap disarm EXIT

echo "Steering sequence on ${IFACE}"
echo "Node order: 1 2 3 4 (rr fl fr rl)"

# Select As5600 angle profile before arming.
send_to_nodes "22" "01"
sleep 0.05

echo "Arm"
send_to_nodes "23" "01"
sleep 0.05

send_sweep "0 -> 90 deg" 0 90000
send_sweep "90 -> 0 deg" 90000 0
hold_angle "hold 0 deg" "00000000" "${HOLD_ZERO_ITERATIONS}"

echo "Disarm"
disarm
trap - EXIT
