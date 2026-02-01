from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    config = os.path.join(
        get_package_share_directory("mr2_spectrophotometer"),
        "config",
        "spectrophotometer.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="mr2_spectrophotometer",
                executable="spectrophotometer_node",
                name="spectrophotometer_node",
                output="screen",
                parameters=[config],
            )
        ]
    )
