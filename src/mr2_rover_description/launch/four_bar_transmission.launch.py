from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


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

    can_iface = LaunchConfiguration("can_iface")
    can_iface_arg = DeclareLaunchArgument(
        "can_iface",
        default_value="can0",
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

    input_limit_can_id = LaunchConfiguration("input_limit_can_id")
    input_limit_can_id_arg = DeclareLaunchArgument(
        "input_limit_can_id",
        default_value="0x183",
        description="CAN ID of the limit switch sensor guarding the input joint",
    )

    output_abs_can_id = LaunchConfiguration("output_abs_can_id")
    output_abs_can_id_arg = DeclareLaunchArgument(
        "output_abs_can_id",
        default_value="0x182",
        description="CAN ID of the absolute encoder on the output joint",
    )

    use_mock_servos = LaunchConfiguration("use_mock_servos")
    use_mock_servos_arg = DeclareLaunchArgument(
        "use_mock_servos",
        default_value="true",
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
            " input_limit_can_id:=",
            input_limit_can_id,
            " output_abs_can_id:=",
            output_abs_can_id,
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
        parameters=[controller_yaml],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    mock_servo_motor_a = Node(
        package="mr2_devices_ak_servo",
        executable="mock_ak_servo_node",
        parameters=[{
            "can_iface": can_iface,
            "motor_id": ParameterValue(motor_a_id, value_type=int),
            "limit_switch_enabled": True,
            "limit_switch_trigger_position": -1.0,
            "limit_switch_trigger_when_below": True,
            "limit_switch_active_high": True,
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
            "absolute_encoder_enabled": True,
        }],
        condition=IfCondition(use_mock_servos),
        output="screen",
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["four_bar_position_controller"],
    )

    return LaunchDescription(
        [
            can_iface_arg,
            motor_a_arg,
            motor_b_arg,
            input_limit_can_id_arg,
            output_abs_can_id_arg,
            use_mock_servos_arg,
            robot_state_publisher_node,
            ros2_control_node,
            mock_servo_motor_a,
            mock_servo_motor_b,
            TimerAction(period=2.0, actions=[jsb_spawner]),
            TimerAction(period=3.0, actions=[controller_spawner]),
        ]
    )
