#!/usr/bin/env bash

set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
ros_workspace="$repo_root/rover/ros2_ws"

if git -C "$repo_root" submodule status | rg -q '^-'; then
  echo "Git submodules are not initialized on the host." >&2
  echo "Run: git submodule update --init --recursive" >&2
  exit 1
fi

echo "Updating apt and rosdep indexes..."
sudo apt-get update
rosdep update --rosdistro humble

echo "Installing ROS workspace dependencies..."
rosdep install \
  --from-paths "$ros_workspace/src" \
  --ignore-src \
  --rosdistro humble \
  -y

echo "Building ROS interfaces required by the gateway..."
cd "$ros_workspace"
colcon build \
  --symlink-install \
  --packages-up-to \
    rtcm_msgs \
    ublox_ubx_msgs

set +u
# shellcheck disable=SC1091
source "$ros_workspace/install/setup.bash"
set -u

echo "Installing gateway dependencies from package-lock.json..."
npm --prefix "$repo_root/base/gateway" ci

echo "Installing dashboard dependencies from package-lock.json..."
npm --prefix "$repo_root/dashboard" ci --include=dev

echo
echo "Development dependencies are ready."
echo "Run ./scripts/dev_container.bash check to build and test the control stack."
