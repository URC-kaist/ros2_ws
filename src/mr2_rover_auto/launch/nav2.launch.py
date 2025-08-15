# .launch.py for Nav2 bringup
# Make sure the estimator.launch.py is launched.
# This has to be launched before action server/client.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("mr2_rover_auto")
    params_file = os.path.join(pkg_share, "config", "nav2_params.yaml")
    nav_to_pose_bt_file = os.path.join(pkg_share, "behavior_trees", "my_navigate_to_pose.xml")
    nav_through_bt_file = os.path.join(pkg_share, "behavior_trees", "my_navigate_through_poses.xml")
    map_file = os.path.join(pkg_share, "maps", "map.yaml")

    return LaunchDescription([
        # For Gazebo, set to true. For field test, set to false.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('pose_xml', default_value=nav_to_pose_bt_file),
        DeclareLaunchArgument('poses_xml', default_value=nav_through_bt_file),
        DeclareLaunchArgument('map', default_value=map_file),

        # 1) BT Navigator: Load BT XML and available node (=class) implementations
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'default_nav_to_pose_bt_xml': LaunchConfiguration('pose_xml')},
                        {'default_nav_through_poses_bt_xml': LaunchConfiguration('poses_xml')}],
        ),

        # 1.5) Behavior Server: For spin, wait, backup, etc.
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),

        # 2) Map Server: Publish pre-baked static occupancy /map
        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'yaml_filename': LaunchConfiguration('map')}],
        ),

        # 3) 4) Costmap nodes are launched by planner and controller servers.

        # 5) Planner: Create path from global costmap; A to B
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 6) Smoother
        Node(
            package='nav2_smoother', executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 7) Controller
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}]
        ),

        # 8) Lifecycle Manager (XXX this has to be the last one to be launched)
        TimerAction( period=2.0,  # Wait for all nodes to be ready
            actions=[
            Node(
                package='nav2_lifecycle_manager', executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': [ # List of nodes to be managed
                    'bt_navigator', 'behavior_server', 'map_server',
                    'planner_server', 'smoother_server', 'controller_server'
                    ]
                }]
            ),
            ]
        ),
    ])
