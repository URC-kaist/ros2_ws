from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "cartesian_command_topic",
            default_value="/moveit_servo/delta_twist_cmds",
            description="TwistStamped command topic configured in MoveIt Servo.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="base_link",
            description="Frame id for TwistStamped header.",
        ),
        DeclareLaunchArgument(
            "linear_step",
            default_value="0.10",
            description="Linear velocity step in m/s applied per key press.",
        ),
        DeclareLaunchArgument(
            "angular_step",
            default_value="0.5",
            description="Angular velocity step in rad/s applied per key press.",
        ),
        DeclareLaunchArgument(
            "publish_rate",
            default_value="50.0",
            description="Publishing rate in Hz.",
        ),
        DeclareLaunchArgument(
            "stop_timeout",
            default_value="0.5",
            description="Seconds after last key press to automatically send zeros.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock; set false for real hardware.",
        ),
    ]

    node = Node(
        package="mr2_servo_teleop",
        executable="servo_keyboard",
        name="servo_keyboard",
        output="screen",
        parameters=[
            {
                "cartesian_command_topic": LaunchConfiguration("cartesian_command_topic"),
                "frame_id": LaunchConfiguration("frame_id"),
                "linear_step": LaunchConfiguration("linear_step"),
                "angular_step": LaunchConfiguration("angular_step"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "stop_timeout": LaunchConfiguration("stop_timeout"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(args + [node])
