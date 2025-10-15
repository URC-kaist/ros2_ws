#!/usr/bin/env bash

set -euo pipefail

sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

ip -details link show can0
