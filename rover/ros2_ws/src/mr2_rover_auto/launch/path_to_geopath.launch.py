"""
Launch GeoPath converters on the base station.

This publishes WGS84 GeoPath topics derived from map-frame Paths so the dashboard
can render plans without per-point /toLL service calls (reduced ROS traffic).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        ),
    ]

    plan_node = Node(
        package="mr2_rover_auto",
        executable="path_to_geopath_node",
        name="plan_path_to_geopath",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "input_topic": "/plan_smoothed",
                "output_topic": "/plan_smoothed/geo",
                "gps_topic": "/gps/filtered",
                "map_frame": "map",
                "utm_frame": "utm",
                "wgs84_frame": "wgs84",
                "transient_local": False,
            }
        ],
    )

    coverage_node = Node(
        package="mr2_rover_auto",
        executable="path_to_geopath_node",
        name="coverage_path_to_geopath",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "input_topic": "/cover_vision/coverage_path",
                "output_topic": "/cover_vision/coverage_path/geo",
                "gps_topic": "/gps/filtered",
                "map_frame": "map",
                "utm_frame": "utm",
                "wgs84_frame": "wgs84",
                "transient_local": True,
            }
        ],
    )

    return LaunchDescription(launch_args + [
        plan_node,
        coverage_node,
    ])
