#!/usr/bin/env bash
# Emit a burst of Makita-style battery frames for testing the battery monitor.
# Usage: ./emulate_battery.bash [can_iface] [period_seconds]

set -euo pipefail

IFACE="${1:-vcan0}"
PERIOD="${2:-1}"

cmd_exists() { command -v "$1" >/dev/null 2>&1; }
if ! cmd_exists cansend; then
  echo "cansend not found (install can-utils)" >&2
  exit 1
fi

echo "Sending frames on ${IFACE} every ${PERIOD}s. Ctrl-C to stop."

while true; do
  cansend "${IFACE}" 300#505AFA00A00F3900    # summary: 80% SoC, 90% health, 25.0°C, 40.00V
  cansend "${IFACE}" 301#A00F020A28000000    # metadata: 4000 mAh/cell, 2P, 10S, pack life 40, cycle MSBs
  cansend "${IFACE}" 310#01D20F0102D20F01    # cells 1–2
  cansend "${IFACE}" 311#03D10F0104D10F01    # cells 3–4
  cansend "${IFACE}" 312#05D00F0106D00F01    # cells 5–6
  cansend "${IFACE}" 313#07CF0F0108CF0F01    # cells 7–8
  cansend "${IFACE}" 314#09CE0F010ACE0F01    # cells 9–10
  sleep "${PERIOD}"
done
