#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'USAGE'
Usage:
  drive_wheel_slow.bash <wheel_index> [velocity_deg_s] [duration_sec] [can_iface]

Wheel index follows the NoFW deployed driving-node order:
  1 = FR / D01 / node 5
  2 = BR / D02 / node 6
  3 = FL / D03 / node 7
  4 = BL / D04 / node 8

Defaults:
  velocity_deg_s = 30
  duration_sec   = 5
  can_iface      = can0

Examples:
  ./scripts/drive_wheel_slow.bash 1
  ./scripts/drive_wheel_slow.bash 3 -20 10 can0
USAGE
}

if [[ $# -lt 1 || $# -gt 4 ]]; then
  usage
  exit 2
fi

if ! command -v cansend >/dev/null 2>&1; then
  echo "cansend not found. Install can-utils." >&2
  exit 1
fi

wheel_index="$1"
velocity_deg_s="${2:-30}"
duration_sec="${3:-5}"
iface="${4:-can0}"

case "${wheel_index}" in
  1)
    node_id=5
    wheel_label="FR"
    ;;
  2)
    node_id=6
    wheel_label="BR/RR"
    ;;
  3)
    node_id=7
    wheel_label="FL"
    ;;
  4)
    node_id=8
    wheel_label="BL/RL"
    ;;
  *)
    usage
    exit 2
    ;;
esac

if ! ip link show "${iface}" >/dev/null 2>&1; then
  echo "CAN interface '${iface}' not found or not accessible." >&2
  exit 1
fi

payload_for_mdeg_s() {
  python3 - "$1" <<'PY'
import struct
import sys

deg_s = float(sys.argv[1])
mdeg_s = round(deg_s * 1000.0)
if mdeg_s < -(2**31) or mdeg_s > 2**31 - 1:
    raise SystemExit("velocity out of int32 mdeg/s range")
print(struct.pack("<i", int(mdeg_s)).hex().upper())
PY
}

send_velocity() {
  local payload="$1"
  cansend "${iface}" "$(printf '%03X' $((0x210 + node_id)))#${payload}"
}

disarm() {
  send_velocity "00000000" || true
  cansend "${iface}" "$(printf '%03X' $((0x230 + node_id)))#00" || true
}

payload="$(payload_for_mdeg_s "${velocity_deg_s}")"
zero_payload="00000000"
profile_id="$(printf '%03X' $((0x220 + node_id)))"
power_id="$(printf '%03X' $((0x230 + node_id)))"

trap disarm EXIT INT TERM

echo "Wheel ${wheel_index} (${wheel_label}, node ${node_id}) on ${iface}"
echo "Commanding ${velocity_deg_s} deg/s for ${duration_sec}s at 20 Hz"

cansend "${iface}" "${profile_id}#00"
sleep 0.05
cansend "${iface}" "${power_id}#01"
sleep 0.05

python3 - "${duration_sec}" <<'PY' | while read -r _; do
import sys
import time

duration = max(0.0, float(sys.argv[1]))
period = 0.05
end = time.monotonic() + duration
while time.monotonic() < end:
    print("tick", flush=True)
    time.sleep(period)
PY
  send_velocity "${payload}"
done

echo "Stopping"
for _ in {1..5}; do
  send_velocity "${zero_payload}"
  sleep 0.05
done

echo "Disarming"
disarm
trap - EXIT INT TERM
