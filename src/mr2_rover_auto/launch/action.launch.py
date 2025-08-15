# .launch.py for bringing up action server, the interface for Nav2 BT Navigator
# Make sure nav2 is launched properly!

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("mr2_rover_auto")
    params_file = os.path.join(pkg_share, "config", "nav2_params.yaml")

    return LaunchDescription([
        # For Gazebo, set to true. For field test, set to false.
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # ros2 action send_goal /gnss_only mr2_action_interface/action/GnssOnly \
        # "{target_latitude: 38.4065, target_longitude: -110.7919}" --feedback
        Node(
            package='mr2_rover_auto',
            executable='gnss_only_server',
            name='gnss_only_server',
            output='screen',
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
        ),

        # Node(
        #     package='mr2_rover_auto',
        #     executable='cover_vision_server',
        #     name='cover_vision_server',
        #     output='screen',
        #     parameters=[
        #         {"use_sim_time": LaunchConfiguration("use_sim_time")}
        #     ],
        # ),
    ])