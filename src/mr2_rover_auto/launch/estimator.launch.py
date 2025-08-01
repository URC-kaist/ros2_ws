from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction 
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('mr2_rover_auto')

    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            arguments=['-0.1', '0.0', '0.0', '0.0', '0.0', '0.0', # left of robot center
                       'base_link', 'imu_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_gps_north',
            arguments=['0.0', '0.1', '-0.1', '0.0', '0.0', '0.0', # behind below of robot center
                       'base_link', 'gps_north_link']
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'ekf.yaml')],
        ),
        TimerAction(period=10.0, actions=[
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[os.path.join(pkg_share, 'config', 'navsat.yaml')],
            remappings=[
                ('imu', '/imu/data'),
                ('gps/fix', '/rover_north/fix'),
            ]
        )
        ])
    ])
