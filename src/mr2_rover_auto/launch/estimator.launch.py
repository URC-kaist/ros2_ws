# This is a modification of:
# https://github.com/ros-navigation/navigation2_tutorials/tree/rolling/nav2_gps_waypoint_follower_demo
# Which was introduced in Nav2 GPS tutorial:
# https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html
# For system with one /fix and one /imu/data. (No wheel odometry)

# Copyright 2018 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("mr2_rover_auto")
    params_file = os.path.join(pkg_share, "config", "estimator_params.yaml")

    return LaunchDescription([
        #### Dual GPS heading
        Node(
            package='mr2_rover_auto',
            executable='gps_heading_imu_node',
            name='gps_heading_imu_node',
            output='screen'
        ),

        #### robot_localization
        # For Gazebo
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        # 0) static_tf
        Node( # Describe IMU mount
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            arguments=['-0.1', '0.0', '0.0', '0.0', '0.0', '0.0', # left of robot center
                       'base_link', 'imu_link']
        ),
        Node( # Describe Northern GPS mount
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_gps_north',
            arguments=['0.0', '0.1', '-0.1', '0.0', '0.0', '0.0', # behind below of robot center
                       'base_link', 'gps_north_link']
        ),
        Node( # Describe tf: odom -> base_link (dummy tf for Nav2)
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom_base_link',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', # Equivalent
                       'odom', 'base_link']
        ),

        # Please consult the graph:
        # https://docs.ros.org/en/noetic/api/robot_localization/html/integrating_gps.html
        # 1) GPS -> odometry/gps
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
            remappings=[
                ("gps/fix", "rover_north/fix"),
            ],
        ),

        # 2) Single EKF: publish tf: map -> odom
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
    ])
