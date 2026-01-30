import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    pkg_share = get_package_share_directory("mr2_rover_auto")
    nav2_launch = os.path.join(pkg_share, "launch", "nav2.launch.py")
    pipeline_launch = os.path.join(pkg_share, "launch", "traversability_pipeline.launch.py")
    action_launch = os.path.join(pkg_share, "launch", "action.launch.py")

    launch_args = [
        DeclareLaunchArgument(
            "mode",
            default_value="sim",
            description="Operating mode: 'sim' or 'real'",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value=PythonExpression(
                ["'true' if '", LaunchConfiguration("mode"), "' == 'sim' else 'false'"]
            ),
            description="Use simulation time; defaults to true in sim mode and false in real",
        ),
        DeclareLaunchArgument(
            "launch_actions",
            default_value="false",
            description="Launch action servers (currently experimental)",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=os.path.join(pkg_share, "maps", "map.yaml"),
            description="Occupancy grid YAML passed to Nav2 map_server",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
            description="Nav2 parameters file",
        ),
    ]

    pipeline_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pipeline_launch),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )

    action_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(action_launch),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_actions")),
    )

    return LaunchDescription(launch_args + [pipeline_include, nav2_include, action_include])
