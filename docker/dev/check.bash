#!/usr/bin/env bash

set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
ros_workspace="$repo_root/rover/ros2_ws"

echo "Running gateway tests..."
npm --prefix "$repo_root/base/gateway" test

echo "Running dashboard typecheck, lint, and production build..."
VITE_OPERATING_PROFILE=rover-direct \
  npm --prefix "$repo_root/dashboard" run check

echo "Building ROS packages needed by direct manual control..."
cd "$ros_workspace"
colcon build \
  --symlink-install \
  --packages-up-to \
    mr2_can_hardware_interface \
    mr2_devices_ak_servo \
    mr2_devices_output_actuator \
    mr2_moveit \
    mr2_rover_control \
    mr2_rover_description \
    mr2_system_status \
    mr2_video_streaming \
    mr2_xbee_bridge

# mr2_launch intentionally has broad runtime dependencies, including autonomy
# packages. Build it separately so the direct-control dependency set remains
# explicit and small.
colcon build \
  --symlink-install \
  --packages-select mr2_launch

echo "Running tests for the packages built above..."
colcon test \
  --packages-select \
    mr2_action_interface \
    mr2_battery_monitor \
    mr2_can_bus_core \
    mr2_can_hardware_interface \
    mr2_devices_ak_servo \
    mr2_devices_output_actuator \
    mr2_launch \
    mr2_moveit \
    mr2_rover_control \
    mr2_rover_description \
    mr2_system_status \
    mr2_video_streaming \
    mr2_xbee_bridge \
    rtcm_msgs \
    ublox_ubx_interfaces \
    ublox_ubx_msgs
colcon test-result --verbose
