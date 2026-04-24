from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
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
    # Launches the single-joint example in boot-origin mode.
    pkg_share = FindPackageShare("mr2_rover_description")
    xacro_file = PathJoinSubstitution([
        pkg_share,
        "urdf",
        "single_joint_absolute_encoder.urdf.xacro",
    ])
    controller_yaml = PathJoinSubstitution([
        pkg_share,
        "config",
        "controllers",
        "single_joint.yaml",
    ])

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time; set true when running in sim or bag replay",
    )

    can_iface, can_iface_arg = declare_can_iface(
        description="CAN interface used by the AK servo hardware",
    )

    motor_id = LaunchConfiguration("motor_id")
    motor_id_arg = DeclareLaunchArgument(
        "motor_id",
        default_value="4",
        description="Motor ID assigned to the single joint actuator",
    )

    use_mock_servo, use_mock_servo_arg = declare_use_mock_servos(
        description="Start mock AK servo that emulates CAN feedback",
    )

    origin_offset = LaunchConfiguration("origin_offset")
    origin_offset_arg = DeclareLaunchArgument(
        "origin_offset",
        default_value="0.0",
        description="Logical joint value assigned to the startup physical pose",
    )

    robot_description = {
        "robot_description": Command([
            "xacro ",
            xacro_file,
            " can_iface:=",
            can_iface,
            " motor_id:=",
            motor_id,
            " origin_offset:=",
            origin_offset,
        ])
    }

    robot_state_publisher_node = make_rsp_node(robot_description, use_sim_time)

    ros2_control_node = make_ros2_control_node(
        controller_yaml,
        robot_description,
        use_sim_time,
    )

    mock_servo_node = Node(
        package="mr2_devices_ak_servo",
        executable="mock_ak_servo_node",
        parameters=[{
            "can_iface": can_iface,
            "motor_id": ParameterValue(motor_id, value_type=int),
            "initial_position": 1.0,
        }],
        condition=IfCondition(use_mock_servo),
        output="screen",
    )

    spawners = controller_spawners(
        ["joint_state_broadcaster", "joint_position_controller"],
        start_after=2.0,
        interval=1.0,
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            can_iface_arg,
            motor_id_arg,
            use_mock_servo_arg,
            origin_offset_arg,
            robot_state_publisher_node,
            mock_servo_node,
            ros2_control_node,
            *spawners,
        ]
    )
