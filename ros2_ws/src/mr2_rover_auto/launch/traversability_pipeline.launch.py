import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mr2_rover_auto")
    filter_chain = os.path.join(pkg_share, "config", "traversability_filter_chain.yaml")

    return LaunchDescription(
        [
            Node(
                package="mr2_rover_auto",
                executable="pc2_to_heightmap_node",
                name="pc2_to_heightmap",
                output="screen",
                parameters=[
                    {
                        "cloud_topic": "/rgbd_camera/points",
                        "base_frame": "base_link",
                        "map_frame": "base_link",
                        "x_forward_m": 5.0,
                        "y_width_m": 3.0,
                        "resolution": 0.05,
                        "layer_name": "elevation",
                    }
                ],
            ),
            Node(
                package="mr2_rover_auto",
                executable="traversability_filter_node",
                name="grid_map_filters",
                output="screen",
                parameters=[filter_chain],
            ),
            Node(
                package="mr2_rover_auto",
                executable="gridmap_to_occupancy_node",
                name="gridmap_to_occupancy",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/traversability_gridmap",
                        "output_topic": "/traversability_occupancy",
                        "layer": "traversability",
                        "min_value": 0.0,
                        "max_value": 1.0,
                        "invert": True,
                    }
                ],
            ),
        ]
    )
