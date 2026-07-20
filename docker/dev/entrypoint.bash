#!/usr/bin/env bash

set -e

# shellcheck disable=SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"

workspace_setup="/workspace/mr2-stack/rover/ros2_ws/install/setup.bash"
if [ -f "$workspace_setup" ]; then
  set +u
  # shellcheck disable=SC1090
  source "$workspace_setup"
  set -u
fi

exec "$@"

