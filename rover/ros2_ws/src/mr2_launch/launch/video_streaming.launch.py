from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from mr2_launch.env import load_mr2_env


def generate_launch_description():
    load_mr2_env(required=["MR2_BASE_IP"])

    video_config_arg = DeclareLaunchArgument(
        "video_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("mr2_launch"), "config", "video_streams.json"]
        ),
        description="Central JSON video stream configuration file",
    )
    video_base_host_arg = DeclareLaunchArgument(
        "video_base_host",
        default_value=EnvironmentVariable("MR2_BASE_IP"),
        description="Base-station host/IP that will receive the rover RTP/UDP video streams",
    )

    video_streaming_node = Node(
        package="mr2_video_streaming",
        executable="video_streaming_node",
        name="video_streaming",
        output="screen",
        parameters=[
            {
                "video_config_path": LaunchConfiguration("video_config"),
                "base_host": LaunchConfiguration("video_base_host"),
            }
        ],
    )

    return LaunchDescription(
        [
            video_config_arg,
            video_base_host_arg,
            video_streaming_node,
        ]
    )
