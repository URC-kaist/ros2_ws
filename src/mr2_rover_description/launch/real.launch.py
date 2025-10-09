from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare("mr2_rover_description")

    xacro_file = PathJoinSubstitution([desc_pkg, "urdf", "rover.urdf.xacro"])
    default_controller = PathJoinSubstitution(
        [desc_pkg, "config", "controllers", "rover_controllers.yaml"]
    )

    can_iface = LaunchConfiguration("can_iface")
    can_iface_arg = DeclareLaunchArgument(
        "can_iface",
        default_value="can0",
        description="CAN interface connected to the rover hardware",
    )

    controller_config = LaunchConfiguration("controller_config")
    controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=default_controller,
        description="YAML file with controller manager configuration",
    )

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                xacro_file,
                " ros2_control_mode:=real_hardware",
                " can_iface:=",
                can_iface,
                " ros2_control_config:=",
                controller_config,
            ]
        )
    }

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config, robot_description],
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    rover_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover_controller"],
        output="screen",
    )
    manipulator_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_controller"],
        output="screen",
    )

    return LaunchDescription(
        [
            can_iface_arg,
            controller_config_arg,
            rsp,
            ros2_control,
            TimerAction(period=2.0, actions=[jsb_spawner]),
            TimerAction(
                period=4.0,
                actions=[rover_controller_spawner, manipulator_controller_spawner],
            ),
        ]
    )
