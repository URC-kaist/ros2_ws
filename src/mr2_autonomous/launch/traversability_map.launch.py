from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('mr2_autonomous'), 'config', 'filter_chain.yaml'
    )

    return LaunchDescription([
        Node(
            package='grid_map_demos',
            executable='filters_demo',
            name='grid_map_filters',
            output='screen',
            parameters=[config_file]
        ),
    ])

