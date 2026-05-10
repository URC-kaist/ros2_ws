#!/usr/bin/env bash
set -euo pipefail

IFACE="${1:-can0}"
BITRATE="${CAN_BITRATE:-500000}"

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# Bring CAN down first (important!)
sudo ip link set "${IFACE}" down || true

# Configure classic CAN at the firmware bitrate.
sudo ip link set "${IFACE}" type can \
  bitrate "${BITRATE}" \
  restart-ms 10

# Bring it up
sudo ip link set "${IFACE}" up

# Show details
ip -details link show "${IFACE}"
