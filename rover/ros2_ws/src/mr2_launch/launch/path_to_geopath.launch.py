"""
Launch GeoPath converters on the base station.

This publishes WGS84 GeoPath topics derived from map-frame Paths so the dashboard
can render plans without per-point /toLL service calls (reduced ROS traffic).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    plan_node = Node(
        package="mr2_base",
        executable="path_to_geopath_node",
        name="plan_path_to_geopath",
        output="screen",
        parameters=[
            {
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
        package="mr2_base",
        executable="path_to_geopath_node",
        name="coverage_path_to_geopath",
        output="screen",
        parameters=[
            {
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

    return LaunchDescription([
        plan_node,
        coverage_node,
    ])
