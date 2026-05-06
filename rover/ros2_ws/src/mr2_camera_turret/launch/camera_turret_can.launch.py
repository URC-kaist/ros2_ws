from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_node(context):
    can_id = int(LaunchConfiguration("can_id").perform(context), 0)
    return [
        Node(
            package="mr2_camera_turret",
            executable="camera_turret_can_node",
            name="camera_turret_can",
            parameters=[
                {
                    "can_iface": LaunchConfiguration("can_iface"),
                    "can_id": can_id,
                    "command_topic": LaunchConfiguration("command_topic"),
                    "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                    "command_timeout_sec": LaunchConfiguration(
                        "command_timeout_sec"
                    ),
                    "invert_x": LaunchConfiguration("invert_x"),
                    "invert_y": LaunchConfiguration("invert_y"),
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("can_iface", default_value="can0"),
            DeclareLaunchArgument("can_id", default_value="0x123"),
            DeclareLaunchArgument(
                "command_topic", default_value="/camera_turret/command"
            ),
            DeclareLaunchArgument("publish_rate_hz", default_value="50.0"),
            DeclareLaunchArgument("command_timeout_sec", default_value="0.5"),
            DeclareLaunchArgument("invert_x", default_value="false"),
            DeclareLaunchArgument("invert_y", default_value="false"),
            OpaqueFunction(function=_launch_node),
        ]
    )
