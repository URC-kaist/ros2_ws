#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace="$repo_root/rover/ros2_ws"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is not installed at /opt/ros/humble." >&2
  exit 1
fi
if [[ ! -f "$workspace/install/setup.bash" ]]; then
  echo "Build the rover workspace before starting direct mode: $workspace" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$workspace/install/setup.bash"
set -u

exec ros2 launch mr2_launch rover_direct.launch.py \
  can_iface:="${MR2_CAN_IFACE:-can0}" \
  controller_spawn_delay:="${MR2_CONTROLLER_SPAWN_DELAY:-10.0}" \
  use_mock_servos:="${MR2_USE_MOCK_SERVOS:-false}" \
  xbee_device:="${MR2_ROVER_XBEE_DEVICE:-/run/mr2/xbee_rover}" \
  start_xbee_sim:=false \
  video_base_host:=127.0.0.1
