#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# Source the ROS 2 installation
source /opt/ros/humble/setup.bash

# Source your ROS 2 workspace
source /home/rover/ros2_ws/install/local_setup.bash

# Launch the ROS 2 launch file
exec ros2 launch mr2_launch real_launch.py
