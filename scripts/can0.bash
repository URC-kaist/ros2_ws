#!/usr/bin/env bash
set -euo pipefail

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# Bring CAN down first (important!)
sudo ip link set can0 down || true

# Configure bitrate
sudo ip link set can0 type can bitrate 1000000

# Bring it up
sudo ip link set can0 up

# Show details
ip -details link show can0
