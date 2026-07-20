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
            [
                FindPackageShare("mr2_rover_description"),
                "config",
                "controllers",
                "rover_controllers.yaml",
            ]
        ),
        description="Controller manager YAML shared by sim and hardware",
    )
    can_iface_arg = DeclareLaunchArgument(
        "can_iface",
        default_value="can0",
        description="CAN interface used by the AK servo hardware",
    )
    enable_manipulator_module_arg = DeclareLaunchArgument(
        "enable_manipulator_module",
        default_value="false",
        description="Enable manipulator URDF and ros2_control in simulation",
    )
    enable_autonomous_module_arg = DeclareLaunchArgument(
        "enable_autonomous_module",
        default_value="true",
        description="Enable autonomous camera module (front_camera) in simulation",
    )
    xbee_sim_device_arg = DeclareLaunchArgument(
        "xbee_sim_device",
        default_value="/tmp/xbee_sim0",
        description="PTy path the XBEE bridge will open in sim mode",
    )
    xbee_sim_peer_arg = DeclareLaunchArgument(
        "xbee_sim_peer",
        default_value="/tmp/xbee_sim1",
        description="Peer PTY path for external attachment",
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
            "enable_manipulator_module_sim": LaunchConfiguration("enable_manipulator_module"),
            "enable_autonomous_module_sim": LaunchConfiguration("enable_autonomous_module"),
            "xbee_sim_device": LaunchConfiguration("xbee_sim_device"),
            "xbee_sim_peer": LaunchConfiguration("xbee_sim_peer"),
        }.items(),
    )

    return LaunchDescription(
        [
            headless_arg,
            rviz_arg,
            controller_config_arg,
            can_iface_arg,
            enable_manipulator_module_arg,
            enable_autonomous_module_arg,
            xbee_sim_device_arg,
            xbee_sim_peer_arg,
            rover_launch,
        ]
    )
