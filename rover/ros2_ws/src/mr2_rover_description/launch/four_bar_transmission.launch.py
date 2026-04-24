from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from mr2_rover_description.launch_common import (
    controller_spawners,
    declare_can_iface,
    declare_use_mock_servos,
    robot_state_publisher_node as make_rsp_node,
    ros2_control_node as make_ros2_control_node,
)


def generate_launch_description():
    pkg_share = FindPackageShare("mr2_rover_description")

    xacro_file = PathJoinSubstitution([
        pkg_share,
        "urdf",
        "four_bar_transmission.urdf.xacro",
    ])
    controller_yaml = PathJoinSubstitution([
        pkg_share,
        "config",
        "controllers",
        "four_bar_transmission.yaml",
    ])

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time; set true when replaying bag files",
    )

    can_iface, can_iface_arg = declare_can_iface(
        description="CAN interface used by the AK servo hardware",
    )

    motor_a_id = LaunchConfiguration("motor_a_id")
    motor_a_arg = DeclareLaunchArgument(
        "motor_a_id",
        default_value="102",
        description="Motor ID assigned to the input joint actuator",
    )

    motor_b_id = LaunchConfiguration("motor_b_id")
    motor_b_arg = DeclareLaunchArgument(
        "motor_b_id",
        default_value="103",
        description="Motor ID assigned to the output joint actuator",
    )

    use_mock_servos, use_mock_servos_arg = declare_use_mock_servos(
        default="true",
        description="Start mock AK servo nodes that emulate the two CAN motors",
    )

    robot_description = {
        "robot_description": Command([
            "xacro ",
            xacro_file,
            " can_iface:=",
            can_iface,
            " motor_a_id:=",
            motor_a_id,
            " motor_b_id:=",
            motor_b_id,
        ])
    }

    robot_state_publisher_node = make_rsp_node(robot_description, use_sim_time)

    ros2_control_node = make_ros2_control_node(
        controller_yaml,
        robot_description,
        use_sim_time,
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )

    mock_servo_motor_a = Node(
        package="mr2_devices_ak_servo",
        executable="mock_ak_servo_node",
        parameters=[{
            "can_iface": can_iface,
            "motor_id": ParameterValue(motor_a_id, value_type=int),
        }],
        condition=IfCondition(use_mock_servos),
        output="screen",
    )

    mock_servo_motor_b = Node(
        package="mr2_devices_ak_servo",
        executable="mock_ak_servo_node",
        parameters=[{
            "can_iface": can_iface,
            "motor_id": ParameterValue(motor_b_id, value_type=int),
        }],
        condition=IfCondition(use_mock_servos),
        output="screen",
    )

    spawners = controller_spawners(
        ["joint_state_broadcaster", "four_bar_position_controller"],
        start_after=2.0,
        interval=1.0,
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            can_iface_arg,
            motor_a_arg,
            motor_b_arg,
            use_mock_servos_arg,
            robot_state_publisher_node,
            ros2_control_node,
            mock_servo_motor_a,
            mock_servo_motor_b,
            *spawners,
        ]
    )
