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
    # Launches the single-joint example that homes using an absolute encoder.
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

    abs_can_id = LaunchConfiguration("absolute_encoder_can_id")
    abs_can_id_arg = DeclareLaunchArgument(
        "absolute_encoder_can_id",
        default_value="384",
        description="CAN ID (decimal) for the absolute encoder device",
    )

    abs_ticks_per_rev = LaunchConfiguration("absolute_encoder_ticks_per_rev")
    abs_ticks_per_rev_arg = DeclareLaunchArgument(
        "absolute_encoder_ticks_per_rev",
        default_value="4096",
        description="Ticks per revolution reported by the absolute encoder",
    )

    abs_direction = LaunchConfiguration("absolute_encoder_direction")
    abs_direction_arg = DeclareLaunchArgument(
        "absolute_encoder_direction",
        default_value="1.0",
        description="Direction multiplier applied to the encoder (+1 or -1)",
    )

    homing_encoder_direction = LaunchConfiguration("homing_encoder_direction")
    homing_encoder_direction_arg = DeclareLaunchArgument(
        "homing_encoder_direction",
        default_value=abs_direction,
        description="Multiplier applied inside the homing policy (+1 or -1)",
    )

    homing_home_offset = LaunchConfiguration("homing_home_offset")
    homing_home_offset_arg = DeclareLaunchArgument(
        "homing_home_offset",
        default_value="0.0",
        description="Offset (rad) added to the encoder angle before computing the joint home",
    )

    robot_description = {
        "robot_description": Command([
            "xacro ",
            xacro_file,
            " can_iface:=",
            can_iface,
            " motor_id:=",
            motor_id,
            " absolute_encoder_can_id:=",
            abs_can_id,
            " absolute_encoder_ticks_per_rev:=",
            abs_ticks_per_rev,
            " absolute_encoder_direction:=",
            abs_direction,
            " homing_encoder_direction:=",
            homing_encoder_direction,
            " homing_home_offset:=",
            homing_home_offset,
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
            "limit_switch_enabled": False,
            "absolute_encoder_enabled": True,
            "absolute_encoder_can_id": ParameterValue(
                abs_can_id, value_type=int),
            "absolute_encoder_ticks_per_rev": ParameterValue(
                abs_ticks_per_rev, value_type=float),
            "absolute_encoder_direction": ParameterValue(
                abs_direction, value_type=float),
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
            abs_can_id_arg,
            abs_ticks_per_rev_arg,
            abs_direction_arg,
            homing_encoder_direction_arg,
            homing_home_offset_arg,
            robot_state_publisher_node,
            mock_servo_node,
            ros2_control_node,
            *spawners,
        ]
    )
