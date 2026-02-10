# .launch.py for dual GPS heading, dual EKF localization, WGS84-to-ENU query, and static tf for Nav2
# Make sure the sensors and their topics (imu/data, */fix) are all up!

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("mr2_rover_auto")
    params_file_sim = os.path.join(pkg_share, "config", "dual_ekf_navsat.yaml")
    params_file_real = os.path.join(pkg_share, "config", "dual_ekf_navsat_real.yaml")

    return LaunchDescription([
        # For Gazebo, set to true. For field test, set to false.
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        #### Dual GPS Global heading calculation
        # 1) compute dual-GNSS yaw and publish into GPS odometry stream
        Node(
            package='mr2_rover_auto',
            executable='gps_heading_gps_node',
            name='gps_heading_gps_node',
            output='screen',
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                # Flip baseline direction to correct 180° heading inversion (north/south swap)
                {"baseline_direction": -1},
            ],
        ),

        # Real robot: set datum from base station survey-in.
        Node(
            package="mr2_rover_auto",
            executable="base_datum_setter",
            name="base_datum_setter",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"require_svin_complete": False},
                {"allow_provisional": True},
                {"allow_fix_fallback": True},
                {"fallback_fix_topic": "/left_gnss/navsat"},
                {"navsat_service": "/navsat_transform/datum"},
            ],
            condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
        ),

        # Wheel encoder odometry from ros2_control joint_states
        # Published by mr2_rover_control::TwistToCommandsController now.

        # Please consult the graph:
        # https://docs.ros.org/en/noetic/api/robot_localization/html/integrating_gps.html
        # 2) GPS -> odometry/gps/raw (navsat_transform output)
        TimerAction(period=2.0, actions=[
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[
                params_file_sim,
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
            remappings=[
                ("/gps/fix", "/left_gnss/navsat"),
                ("/odometry/gps", "/odometry/gps/raw"),
                ('/odometry/filtered', '/odometry/filtered/global'),
                ("datum", "/navsat_transform/datum"),
            ],
        ),

        # 3) Local EKF: publish tf: odom -> base_link
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_local",
            output="screen",
            parameters=[
                params_file_sim,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered/local')
            ],
        ),

        # 4) Global EKF: publish tf: map -> odom
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_global",
            output="screen",
            parameters=[
                params_file_sim,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered/global')
            ],
        ),

        ],
        condition=IfCondition(LaunchConfiguration("use_sim_time"))),

        TimerAction(period=2.0, actions=[
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[
                params_file_real,
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
            remappings=[
                ("/gps/fix", "/left_gnss/navsat"),
                ("/odometry/gps", "/odometry/gps/raw"),
                ('/odometry/filtered', '/odometry/filtered/global'),
                ("datum", "/navsat_transform/datum"),
            ],
        ),

        # 3) Local EKF: publish tf: odom -> base_link
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
                ('/odometry/filtered', '/odometry/filtered/local')
            ],
        ),

        # 4) Global EKF: publish tf: map -> odom
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_global",
            output="screen",
            parameters=[
                params_file_real,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ('/odometry/filtered', '/odometry/filtered/global')
            ],
        ),

        ],
        condition=UnlessCondition(LaunchConfiguration("use_sim_time"))),
    ])
