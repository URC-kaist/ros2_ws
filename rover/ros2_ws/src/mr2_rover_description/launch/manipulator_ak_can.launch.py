from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from mr2_rover_description.launch_common import (
    controller_spawners,
    declare_can_iface,
    declare_controller_config,
    declare_use_mock_servos,
    mock_servo_nodes,
    robot_description_from_xacro,
    robot_state_publisher_node,
    ros2_control_node,
)


def generate_launch_description():
    desc_pkg = FindPackageShare("mr2_rover_description")

    xacro_file = PathJoinSubstitution(
        [desc_pkg, "urdf", "manipulator_ak_can.urdf.xacro"]
    )
    default_controller = PathJoinSubstitution(
        [desc_pkg, "config", "controllers", "manipulator_controllers.yaml"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time; normally false unless replaying data",
    )

    can_iface, can_iface_arg = declare_can_iface(
        description="CAN interface connected to the manipulator hardware",
    )

    controller_config, controller_config_arg = declare_controller_config(
        default_value=default_controller,
        description="YAML file with controller manager configuration for the manipulator",
    )

    use_mock_servos, use_mock_servos_arg = declare_use_mock_servos(
        description="Start mock AK servo nodes that emulate the manipulator CAN motors",
    )

    robot_description = robot_description_from_xacro(
        xacro_file,
        {
            "can_iface": can_iface,
        },
    )

    rsp = robot_state_publisher_node(robot_description, use_sim_time)

    mock_servos = mock_servo_nodes(
        can_iface,
        motor_ids=range(1, 7),
        condition=IfCondition(use_mock_servos),
    )

    ros2_control = ros2_control_node(controller_config, robot_description, use_sim_time)
    spawners = controller_spawners(
        ["joint_state_broadcaster", "manipulator_controller"],
        start_after=2.0,
        interval=2.0,
        inactive_controllers=["manipulator_controller"],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            can_iface_arg,
            controller_config_arg,
            use_mock_servos_arg,
            rsp,
            *mock_servos,
            ros2_control,
            *spawners,
        ]
    )
