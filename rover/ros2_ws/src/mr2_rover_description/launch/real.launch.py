from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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

    xacro_file = PathJoinSubstitution([desc_pkg, "urdf", "rover.urdf.xacro"])
    default_controller = PathJoinSubstitution(
        [desc_pkg, "config", "controllers", "rover_controllers.yaml"]
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time; normally false for hardware",
    )

    can_iface, can_iface_arg = declare_can_iface(
        description="CAN interface connected to the rover hardware",
    )

    controller_config, controller_config_arg = declare_controller_config(
        default_value=default_controller,
        description="YAML file with controller manager configuration",
    )

    use_mock_servos, use_mock_servos_arg = declare_use_mock_servos(
        description="Start mock AK servo nodes that emulate the manipulator CAN motors",
    )

    robot_description = robot_description_from_xacro(
        xacro_file,
        {
            "ros2_control_mode": "real_hardware",
            "can_iface": can_iface,
            "ros2_control_config": controller_config,
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
        # ["joint_state_broadcaster", "rover_controller", "manipulator_controller", "gripper_position_controller"],
        ["joint_state_broadcaster", "manipulator_controller", "gripper_position_controller"],
        start_after=2.0,
        interval=2.0,
        inactive_controllers=["manipulator_controller"],
    )

    battery_monitor = Node(
        package="mr2_battery_monitor",
        executable="battery_monitor_node",
        name="battery_1",
        output="screen",
        parameters=[{"can_iface": can_iface}],
        remappings=[
            ("battery/telemetry", "battery_1/telemetry"),
            ("battery/state", "battery_1/state"),
        ],
    )

    battery_monitor_secondary = Node(
        package="mr2_battery_monitor",
        executable="battery_monitor_node",
        name="battery_2",
        output="screen",
        parameters=[
            {
                "can_iface": can_iface,
                "summary_can_id": 0x320,
                "metadata_can_id": 0x321,
                "cell_base_can_id": 0x330,
            }
        ],
        remappings=[
            ("battery/telemetry", "battery_2/telemetry"),
            ("battery/state", "battery_2/state"),
        ],
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
            battery_monitor,
            battery_monitor_secondary,
        ]
    )
