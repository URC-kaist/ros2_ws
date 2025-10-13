from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare("mr2_rover_description")

    default_controller = PathJoinSubstitution(
        [desc_pkg, "config", "controllers", "manipulator_controllers.yaml"]
    )

    can_iface = LaunchConfiguration("can_iface")
    controller_config = LaunchConfiguration("controller_config")
    use_mock_servos = LaunchConfiguration("use_mock_servos")

    can_iface_arg = DeclareLaunchArgument(
        "can_iface",
        default_value="can0",
        description="CAN interface connected to the manipulator hardware",
    )
    controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=default_controller,
        description="Controller manager YAML for the manipulator ros2_control node",
    )
    use_mock_servos_arg = DeclareLaunchArgument(
        "use_mock_servos",
        default_value="false",
        description="Start mock AK servo nodes instead of hardware interfaces",
    )

    manipulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [desc_pkg, "launch", "manipulator_ak_can.launch.py"]
            )
        ),
        launch_arguments={
            "can_iface": can_iface,
            "controller_config": controller_config,
            "use_mock_servos": use_mock_servos,
        }.items(),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_moveit"), "launch", "move_group.launch.py"]
            )
        )
    )

    return LaunchDescription(
        [
            can_iface_arg,
            controller_config_arg,
            use_mock_servos_arg,
            manipulator_launch,
            move_group_launch,
        ]
    )
