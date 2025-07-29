from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('mr2_heightmap'), 'config', 'filter_chain.yaml'
    )

    return LaunchDescription([
        Node(
            package='grid_map_demos',
            executable='filters_demo',
            name='filters_demo',
            output='screen',
            parameters=[
                {'input_topic': '/height_gridmap'},  # adjust as needed
                {'output_topic': '/traversibility_gridmap'}, # adjust as needed
                config_file  # this loads the full YAML as ROS params
            ]
        ),
    ])

