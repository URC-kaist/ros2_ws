from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction 
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('mr2_rover_auto')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # 0) Lifecycle Manager (automates all transitions)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': [
                'map_server',
                'global_costmap',
                'local_costmap',
                'planner_server',
                'controller_server',
                'smoother_server',
                'bt_navigator']}]
        ),

        # 1) BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 2) Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 3) Global Costmap
        Node(
            package='nav2_costmap_2d',
            executable='costmap_server',
            name='global_costmap',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 4) Local Costmap
        Node(
            package='nav2_costmap_2d',
            executable='costmap_server',
            name='local_costmap',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 5) Planner
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 6) Smoother
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 7) Controller
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
    ])
