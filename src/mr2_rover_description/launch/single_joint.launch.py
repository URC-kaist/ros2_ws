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
        "single_joint.urdf.xacro",
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


    limit_switch_trigger_position_rad = LaunchConfiguration(
        "limit_switch_trigger_position_rad"
    )
    limit_switch_trigger_position_rad_arg = DeclareLaunchArgument(
        "limit_switch_trigger_position_rad",
        default_value="0.0",
        description="Joint position (rad) that triggers the limit switch",
    )

    limit_switch_trigger_when_below = LaunchConfiguration(
        "limit_switch_trigger_when_below"
    )
    limit_switch_trigger_when_below_arg = DeclareLaunchArgument(
        "limit_switch_trigger_when_below",
        default_value="true",
        description="Trigger when position is below threshold (false means above)",
    )

    robot_description = {
        "robot_description": Command([
            "xacro ",
            xacro_file,
            " can_iface:=",
            can_iface,
            " motor_id:=",
            motor_id,
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
            "initial_position_rad": 1.0,
            "limit_switch_enabled": True,
            "limit_switch_trigger_position_rad": ParameterValue(
                limit_switch_trigger_position_rad, value_type=float),
            "limit_switch_trigger_when_below": ParameterValue(
                limit_switch_trigger_when_below, value_type=bool),
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
            limit_switch_trigger_position_rad_arg,
            limit_switch_trigger_when_below_arg,
            robot_state_publisher_node,
            mock_servo_node,
            ros2_control_node,
            TimerAction(period=2.0, actions=[jsb_spawner]),
            TimerAction(period=3.0, actions=[joint_spawner]),
        ]
    )
