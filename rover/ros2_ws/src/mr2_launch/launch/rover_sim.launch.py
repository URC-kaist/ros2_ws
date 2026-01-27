from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run without Gazebo or RViz GUI windows",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("mr2_launch"), "rviz", "sim.rviz"]
        ),
        description="Full path to RViz2 config file",
    )
    controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("mr2_rover_description"), "config", "controllers", "rover_controllers.yaml"]
        ),
        description="Controller manager YAML shared by sim and hardware",
    )
    can_iface_arg = DeclareLaunchArgument(
        "can_iface",
        default_value="can0",
        description="CAN interface used by the AK servo hardware",
    )
    sik_sim_device_arg = DeclareLaunchArgument(
        "sik_sim_device",
        default_value="/tmp/sik_sim0",
        description="PTy path the SiK bridge will open in sim mode",
    )
    sik_sim_peer_arg = DeclareLaunchArgument(
        "sik_sim_peer",
        default_value="/tmp/sik_sim1",
        description="Peer PTY path for external attachment",
    )
    sik_sim_baud_arg = DeclareLaunchArgument(
        "sik_sim_baud",
        default_value="57600",
        description="Baud rate for the simulated SiK link",
    )

    rover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "rover.launch.py"]
            )
        ),
        launch_arguments={
            "mode": "sim",
            "headless": LaunchConfiguration("headless"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "controller_config": LaunchConfiguration("controller_config"),
            "can_iface": LaunchConfiguration("can_iface"),
            "sik_sim_device": LaunchConfiguration("sik_sim_device"),
            "sik_sim_peer": LaunchConfiguration("sik_sim_peer"),
            "sik_sim_baud": LaunchConfiguration("sik_sim_baud"),
        }.items(),
    )

    return LaunchDescription(
        [
            headless_arg,
            rviz_arg,
            controller_config_arg,
            can_iface_arg,
            sik_sim_device_arg,
            sik_sim_peer_arg,
            sik_sim_baud_arg,
            rover_launch,
        ]
    )
