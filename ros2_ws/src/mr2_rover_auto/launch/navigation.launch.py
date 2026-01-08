import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("mr2_rover_auto")
    nav2_launch = os.path.join(pkg_share, "launch", "nav2.launch.py")
    pipeline_launch = os.path.join(pkg_share, "launch", "traversability_pipeline.launch.py")
    action_launch = os.path.join(pkg_share, "launch", "action.launch.py")

    launch_args = [
        DeclareLaunchArgument(
            "launch_actions",
            default_value="false",
            description="Launch action servers (currently experimental)",
        )
    ]

    pipeline_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pipeline_launch),
    )

    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
    )

    action_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(action_launch),
        condition=IfCondition(LaunchConfiguration("launch_actions")),
    )

    return LaunchDescription(launch_args + [pipeline_include, nav2_include, action_include])
