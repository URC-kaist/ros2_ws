from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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

    can_iface = LaunchConfiguration("can_iface")
    can_iface_arg = DeclareLaunchArgument(
        "can_iface",
        default_value="can0",
        description="CAN interface used by the AK servo hardware",
    )

    motor_id = LaunchConfiguration("motor_id")
    motor_id_arg = DeclareLaunchArgument(
        "motor_id",
        default_value="4",
        description="Motor ID assigned to the single joint actuator",
    )

    use_mock_servo = LaunchConfiguration("use_mock_servo")
    use_mock_servo_arg = DeclareLaunchArgument(
        "use_mock_servo",
        default_value="false",
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

    abs_zero_offset = LaunchConfiguration("absolute_encoder_zero_offset")
    abs_zero_offset_arg = DeclareLaunchArgument(
        "absolute_encoder_zero_offset",
        default_value="0.0",
        description="Mechanical zero offset (rad) applied to the encoder",
    )

    abs_direction = LaunchConfiguration("absolute_encoder_direction")
    abs_direction_arg = DeclareLaunchArgument(
        "absolute_encoder_direction",
        default_value="1.0",
        description="Direction multiplier applied to the encoder (+1 or -1)",
    )

    homing_home_position = LaunchConfiguration("homing_home_position")
    homing_home_position_arg = DeclareLaunchArgument(
        "homing_home_position",
        default_value="0.0",
        description="Joint position command (rad) after homing completes",
    )

    homing_encoder_direction = LaunchConfiguration("homing_encoder_direction")
    homing_encoder_direction_arg = DeclareLaunchArgument(
        "homing_encoder_direction",
        default_value=abs_direction,
        description="Multiplier applied inside the homing policy (+1 or -1)",
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
            " absolute_encoder_zero_offset:=",
            abs_zero_offset,
            " absolute_encoder_direction:=",
            abs_direction,
            " homing_home_position:=",
            homing_home_position,
            " homing_encoder_direction:=",
            homing_encoder_direction,
        ])
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_yaml, robot_description],
        output="screen",
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
            "absolute_encoder_zero_offset": ParameterValue(
                abs_zero_offset, value_type=float),
        }],
        condition=IfCondition(use_mock_servo),
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    joint_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_position_controller"],
    )

    return LaunchDescription(
        [
            can_iface_arg,
            motor_id_arg,
            use_mock_servo_arg,
            abs_can_id_arg,
            abs_ticks_per_rev_arg,
            abs_zero_offset_arg,
            abs_direction_arg,
            homing_home_position_arg,
            homing_encoder_direction_arg,
            robot_state_publisher_node,
            mock_servo_node,
            ros2_control_node,
            TimerAction(period=2.0, actions=[jsb_spawner]),
            TimerAction(period=3.0, actions=[joint_spawner]),
        ]
    )
