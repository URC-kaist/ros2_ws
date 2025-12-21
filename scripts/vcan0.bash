#!/usr/bin/env bash

set -euo pipefail

sudo modprobe vcan

if ip link show vcan0 >/dev/null 2>&1; then
  sudo ip link set vcan0 down || true
  sudo ip link del vcan0 || true
fi

sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up

ip -details link show vcan0
