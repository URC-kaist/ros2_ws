# .launch.py for bringing up action server, the interface for Nav2 BT Navigator
# Make sure nav2 is launched properly!

from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    ros2 action send_goal /gnss_only mr2_action_interface/action/GnssOnly "{target_latitude: 38.4065, target_longitude: -110.7919}" --feedback
    """

    pkg_share = get_package_share_directory('mr2_rover_auto')
    mr2_panorama_share = get_package_share_directory('mr2_panorama')
    cover_vision_params = os.path.join(pkg_share, 'config', 'cover_vision_params.yaml')
    front_camera_calibration = os.path.join(
        mr2_panorama_share, 'config', 'front_camera_calibration.yaml')

    return LaunchDescription([
        # For Gazebo, set to true. For field test, set to false.
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('cover_vision_params', default_value=cover_vision_params),
        DeclareLaunchArgument('panorama_video_device', default_value='/dev/videoFRONT'),
        DeclareLaunchArgument('panorama_calibration_yaml', default_value=front_camera_calibration),
        DeclareLaunchArgument('panorama_fixed_frame', default_value='map'),
        DeclareLaunchArgument('panorama_base_frame', default_value='base_link'),

        Node(
            package='mr2_rover_auto',
            executable='gnss_only_server',
            name='gnss_only_server',
            output='screen',
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
        ),

        Node(
            package='mr2_rover_auto',
            executable='cover_vision_server',
            name='cover_vision_server',
            output='screen',
            parameters=[
                LaunchConfiguration("cover_vision_params"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
        ),

        Node(
            package='mr2_panorama',
            executable='panorama_server',
            name='panorama_server',
            output='screen',
            parameters=[
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "video_device": LaunchConfiguration("panorama_video_device"),
                    "calibration_yaml": LaunchConfiguration("panorama_calibration_yaml"),
                    "fixed_frame": LaunchConfiguration("panorama_fixed_frame"),
                    "base_frame": LaunchConfiguration("panorama_base_frame"),
                }
            ],
        ),

        Node(
            package='mr2_rover_auto',
            executable='cover_vision_yolo_adapter',
            name='cover_vision_yolo_adapter',
            output='screen',
            parameters=[
                LaunchConfiguration("cover_vision_params"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
        ),

        Node(
            package='mr2_rover_auto',
            executable='cover_vision_aruco_adapter',
            name='cover_vision_aruco_adapter',
            output='screen',
            parameters=[
                LaunchConfiguration("cover_vision_params"),
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
        ),

        Node(
            package='mr2_rover_auto',
            executable='mission_master',
            name='mission_master',
            output='screen',
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
        ),

        Node(
            package='mr2_rover_auto',
            executable='map_cropper',
            name='map_cropper',
            output='screen',
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
        ),
    ])
