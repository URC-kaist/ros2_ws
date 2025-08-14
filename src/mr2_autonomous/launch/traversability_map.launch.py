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

        Node(
            package="mr2_autonomous",
            executable="gridmap_to_occupancy",
            name="gridmap_to_occupancy",
            output="screen",
            parameters=[{
                "input_topic": "/traversability_gridmap",  # your filters’ output
                "output_topic": "/traversability_occupancy",
                "layer": "traversability",                 # set to your actual layer id
                "min_value": 0.0,
                "max_value": 1.0,
                "invert": True,                            # good(1)→low cost; bad(0)→high cost
            }],
        )
    ])

