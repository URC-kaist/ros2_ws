#!/usr/bin/env bash

set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ros_workspace="$repo_root/rover/ros2_ws"

if [[ -f $repo_root/.nvmrc ]]; then
  IFS= read -r pinned_node_version <"$repo_root/.nvmrc" || true
  pinned_node_version=${pinned_node_version#v}
  nvm_dir=${NVM_DIR:-${HOME:-}/.nvm}
  pinned_node_bin_dir="$nvm_dir/versions/node/v$pinned_node_version/bin"
  if [[ -x $pinned_node_bin_dir/node ]]; then
    export PATH="$pinned_node_bin_dir:$PATH"
  fi
fi

required_commands=(colcon node npm xmllint)
for command_name in "${required_commands[@]}"; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "Required native dependency is missing: $command_name" >&2
    exit 1
  fi
done

node_major=$(node -p 'process.versions.node.split(".")[0]')
if [[ $node_major != 24 ]]; then
  echo "Node.js 24 is required; found $(node --version)." >&2
  exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is not installed at /opt/ros/humble." >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
if [[ -f $ros_workspace/install/setup.bash ]]; then
  # shellcheck disable=SC1091
  source "$ros_workspace/install/setup.bash"
fi
set -u

echo "Running gateway tests with $(node --version)..."
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
    mr2_launch \
    mr2_moveit \
    mr2_rover_control \
    mr2_rover_description \
    mr2_system_status \
    mr2_video_streaming \
    mr2_xbee_bridge

# mr2_launch also owns non-direct launch files and therefore declares
# GNSS/science runtime packages. Colcon builds those workspace dependencies
# even though rover_direct.launch.py does not start them.

echo "Running tests for the packages built above..."
echo "Validating workspace XML syntax locally..."
while IFS= read -r -d '' xml_file; do
  xmllint --noout "$xml_file"
done < <(find "$ros_workspace/src" -type f -name '*.xml' -print0)

# ament_xmllint fetches the ROS package schema over HTTP. Skip that network-
# dependent test here because XML well-formedness was checked locally above.
test_result_base=$(mktemp -d /tmp/mr2-rover-direct-test-results.XXXXXX)
cleanup() {
  rm -rf "$test_result_base"
}
trap cleanup EXIT
colcon test \
  --test-result-base "$test_result_base" \
  --packages-select \
    mr2_action_interface \
    mr2_battery_monitor \
    mr2_can_bus_core \
    mr2_can_hardware_interface \
    mr2_devices_ak_servo \
    mr2_devices_output_actuator \
    mr2_launch \
    mr2_moveit \
    mr2_rover_description \
    mr2_system_status \
    mr2_xbee_bridge \
    rtcm_msgs \
    ublox_ubx_interfaces \
    ublox_ubx_msgs \
  --ctest-args -E xmllint

# These existing C++ packages are not yet formatted with the repository's
# configured ament_uncrustify style. Keep their functional static checks while
# avoiding an unrelated repository-wide source reformat.
colcon test \
  --test-result-base "$test_result_base" \
  --packages-select \
    mr2_rover_control \
    mr2_video_streaming \
  --ctest-args -E 'uncrustify|xmllint'

colcon test-result --test-result-base "$test_result_base" --verbose
