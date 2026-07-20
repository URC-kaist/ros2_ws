"""
Run the manually operated rover stack entirely on the Jetson.

This wrapper deliberately bypasses rover_real.launch.py because that wrapper
owns the GNSS, NTRIP, localization, autonomous-navigation, and science modules.
The base gateway runs as a separate native process on the same Jetson.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from mr2_launch.env import load_mr2_env


def generate_launch_description():
    load_mr2_env()

    can_iface_arg = DeclareLaunchArgument(
        "can_iface",
        default_value="can0",
        description="SocketCAN interface connected to the rover hardware",
    )
    controller_spawn_delay_arg = DeclareLaunchArgument(
        "controller_spawn_delay",
        default_value="10.0",
        description="Delay before spawning ros2_control controllers",
    )
    use_mock_servos_arg = DeclareLaunchArgument(
        "use_mock_servos",
        default_value="false",
        description="Use mock manipulator servo devices instead of CAN hardware",
    )
    xbee_device_arg = DeclareLaunchArgument(
        "xbee_device",
        default_value="/run/mr2/xbee_rover",
        description="Rover side of the local XBEE simulation PTY pair",
    )
    xbee_gateway_device_arg = DeclareLaunchArgument(
        "xbee_gateway_device",
        default_value="/run/mr2/xbee_gateway",
        description="Gateway side of the local XBEE simulation PTY pair",
    )
    start_xbee_sim_arg = DeclareLaunchArgument(
        "start_xbee_sim",
        default_value="false",
        description="Create the local PTY pair in this launch; production systemd owns it",
    )
    video_base_host_arg = DeclareLaunchArgument(
        "video_base_host",
        default_value="127.0.0.1",
        description="Loopback destination used by the co-located video gateway",
    )
    video_config_arg = DeclareLaunchArgument(
        "video_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("mr2_launch"), "config", "video_streams.json"]
        ),
        description="Shared rover/gateway video stream configuration",
    )

    xbee_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "xbee_sim.launch.py"]
            )
        ),
        launch_arguments={
            "enable_xbee_sim": LaunchConfiguration("start_xbee_sim"),
            "xbee_sim_device": LaunchConfiguration("xbee_device"),
            "xbee_sim_peer": LaunchConfiguration("xbee_gateway_device"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_xbee_sim")),
    )

    rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "rover.launch.py"]
            )
        ),
        launch_arguments={
            "mode": "real",
            "use_sim_time": "false",
            "headless": "true",
            "can_iface": LaunchConfiguration("can_iface"),
            "controller_spawn_delay": LaunchConfiguration("controller_spawn_delay"),
            "use_mock_servos": LaunchConfiguration("use_mock_servos"),
            "enable_manipulator_module": "true",
            "enable_autonomous_module": "false",
            "enable_localization": "false",
            "start_manipulator_controllers_active": "true",
            "enable_aruco": "false",
            "xbee_device": LaunchConfiguration("xbee_device"),
            "enable_video_streaming": "true",
            "video_base_host": LaunchConfiguration("video_base_host"),
            "video_config": LaunchConfiguration("video_config"),
        }.items(),
    )

    # When start_xbee_sim=true, give socat time to create both symlinks before
    # the bridge opens its endpoint. The same short delay is harmless under
    # systemd, where the PTY service is already active.
    delayed_rover = TimerAction(period=1.0, actions=[rover])

    return LaunchDescription(
        [
            can_iface_arg,
            controller_spawn_delay_arg,
            use_mock_servos_arg,
            xbee_device_arg,
            xbee_gateway_device_arg,
            start_xbee_sim_arg,
            video_base_host_arg,
            video_config_arg,
            xbee_sim,
            delayed_rover,
        ]
    )
