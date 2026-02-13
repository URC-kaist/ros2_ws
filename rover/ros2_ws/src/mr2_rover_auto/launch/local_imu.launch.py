# Minimal IMU-only localization: publish odom -> base_link and a static map -> odom.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("mr2_rover_auto")
    params_file_real = os.path.join(pkg_share, "config", "ekf_imu_real.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        # Align D435i optical frame to ROS2 ENU frame
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="d435i_filter",
            output="screen",
            parameters=[
                params_file_real,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ("imu/data_raw", "/rgbd_camera/imu"),
                ("imu/data", "/rgbd_camera/imu/filtered"),
            ],
        ),

        # Local EKF: publish tf: odom -> base_link
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_local",
            output="screen",
            parameters=[
                params_file_real,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ('/rgbd_camera/imu', '/rgbd_camera/imu/filtered'),
                ('/odometry/filtered', '/odometry/filtered/local')
            ],
        ),

        # Static identity TF: map -> odom
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_static_tf",
            output="screen",
            arguments=[
                "--x", "0",
                "--y", "0",
                "--z", "0",
                "--roll", "0",
                "--pitch", "0",
                "--yaw", "0",
                "--frame-id", "map",
                "--child-frame-id", "odom",
            ],
        ),
    ])
