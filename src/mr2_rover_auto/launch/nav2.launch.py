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

        # 0) Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': [
                'bt_navigator',
                'map_server',
                'global_costmap',
                'planner_server',
                'controller_server',
                'smoother_server']}]
        ),

        # 1) BT Navigator: Load BT XML and available node (=class) implementations
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 2) Map Server: Publish pre-baked static occupancy /map
        Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        # 3) Global Costmap: /map + TraversabilityLayer (or ObstacleLayer) + InflationLayer
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            name='global_costmap',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 4) Planner: Create path from global costmap; A to B
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_a2b_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),
        """ Once active, you can send a goal with:
        ros2 action send_goal /compute_path_to_pose \
        nav2_msgs/action/ComputePathToPose \
        "{goal: { header: { frame_id: 'map' }, pose: { position: { x: 1.0, y: 2.0 }, orientation: { w: 1.0 } } }}"
        """

        # 5) Planner: Create path from global costmap; Coverage
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_c_server',
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
