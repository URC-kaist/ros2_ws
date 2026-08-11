from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from mr2_launch.env import load_mr2_env


def generate_launch_description():
    load_mr2_env()

    video_config_arg = DeclareLaunchArgument(
        "video_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("mr2_launch"), "config", "video_streams.json"]
        ),
        description="Central JSON video stream configuration file",
    )
    video_base_host_arg = DeclareLaunchArgument(
        "video_base_host",
        default_value=EnvironmentVariable("MR2_BASE_IP", default_value="127.0.0.1"),
        description="Base-station host/IP that will receive the rover RTP/UDP video streams",
    )
    disabled_stream_ids_arg = DeclareLaunchArgument(
        "disabled_stream_ids",
        default_value="",
        description="Comma-separated stream IDs to skip from the central video config",
    )
    stream_lease_timeout_arg = DeclareLaunchArgument(
        "stream_lease_timeout_s",
        default_value="60.0",
        description="Maximum automatic latency trial stream lease duration",
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
                "disabled_stream_ids": LaunchConfiguration("disabled_stream_ids"),
                "stream_lease_timeout_s": LaunchConfiguration("stream_lease_timeout_s"),
            }
        ],
    )

    return LaunchDescription(
        [
            video_config_arg,
            video_base_host_arg,
            disabled_stream_ids_arg,
            stream_lease_timeout_arg,
            video_streaming_node,
        ]
    )
