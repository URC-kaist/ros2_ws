from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

"""
LaunchDescription: the container of everything to do in this launch.
DeclareLaunchArgument: a specific action inside that container,
used to expose configurable parameters to the user.
"""

def generate_launch_description():
    # Declare args for parameter file and simulation time
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            os.getenv('HOME'), 'ros2_ws', 'src', 'mr2_rover_auto', 'config', 'nav2_params.yaml'
        ),
        description='Full path to the Nav2 parameters file.'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Launch Nav2 components
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[LaunchConfiguration('params_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        declare_params,
        declare_use_sim_time,
        planner_server,
        smoother_server,
        controller_server
    ])
