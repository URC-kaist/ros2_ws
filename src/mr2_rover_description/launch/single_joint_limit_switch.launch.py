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
    # Launches the single-joint example that homes using a limit switch.
    pkg_share = FindPackageShare("mr2_rover_description")
    xacro_file = PathJoinSubstitution([
        pkg_share,
        "urdf",
        "single_joint_limit_switch.urdf.xacro",
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

    limit_switch_can_id = LaunchConfiguration("limit_switch_can_id")
    limit_switch_can_id_arg = DeclareLaunchArgument(
        "limit_switch_can_id",
        default_value="385",
        description="CAN ID (decimal) for the limit switch device",
    )

    limit_switch_active_high = LaunchConfiguration("limit_switch_active_high")
    limit_switch_active_high_arg = DeclareLaunchArgument(
        "limit_switch_active_high",
        default_value="true",
        description="Whether the limit switch line is active-high",
    )

    limit_switch_trigger_position = LaunchConfiguration(
        "limit_switch_trigger_position")
    limit_switch_trigger_position_arg = DeclareLaunchArgument(
        "limit_switch_trigger_position",
        default_value="0.0",
        description="Joint position (rad) that triggers the limit switch",
    )

    limit_switch_trigger_when_below = LaunchConfiguration(
        "limit_switch_trigger_when_below")
    limit_switch_trigger_when_below_arg = DeclareLaunchArgument(
        "limit_switch_trigger_when_below",
        default_value="true",
        description="Trigger when position is below threshold (false means above)",
    )

    homing_search_direction = LaunchConfiguration("homing_search_direction")
    homing_search_direction_arg = DeclareLaunchArgument(
        "homing_search_direction",
        default_value="negative",
        description="Direction to move during the initial homing search",
    )

    homing_approach_speed = LaunchConfiguration("homing_approach_speed")
    homing_approach_speed_arg = DeclareLaunchArgument(
        "homing_approach_speed",
        default_value="0.30",
        description="Approach speed (rad/s) used when driving towards the limit",
    )

    homing_backoff_speed = LaunchConfiguration("homing_backoff_speed")
    homing_backoff_speed_arg = DeclareLaunchArgument(
        "homing_backoff_speed",
        default_value="0.60",
        description="Backoff speed (rad/s) used after the switch trips",
    )

    homing_fine_speed = LaunchConfiguration("homing_fine_speed")
    homing_fine_speed_arg = DeclareLaunchArgument(
        "homing_fine_speed",
        default_value="0.03",
        description="Fine approach speed (rad/s) used during the final pass",
    )

    homing_backoff_distance = LaunchConfiguration("homing_backoff_distance")
    homing_backoff_distance_arg = DeclareLaunchArgument(
        "homing_backoff_distance",
        default_value="0.10",
        description="Distance (rad) to back off once the switch is detected",
    )

    robot_description = {
        "robot_description": Command([
            "xacro ",
            xacro_file,
            " can_iface:=",
            can_iface,
            " motor_id:=",
            motor_id,
            " limit_switch_can_id:=",
            limit_switch_can_id,
            " limit_switch_active_high:=",
            limit_switch_active_high,
            " homing_search_direction:=",
            homing_search_direction,
            " homing_approach_speed:=",
            homing_approach_speed,
            " homing_backoff_speed:=",
            homing_backoff_speed,
            " homing_fine_speed:=",
            homing_fine_speed,
            " homing_backoff_distance:=",
            homing_backoff_distance,
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
            "limit_switch_enabled": True,
            "limit_switch_can_id": ParameterValue(
                limit_switch_can_id, value_type=int),
            "limit_switch_active_high": ParameterValue(
                limit_switch_active_high, value_type=bool),
            "limit_switch_trigger_position": ParameterValue(
                limit_switch_trigger_position, value_type=float),
            "limit_switch_trigger_when_below": ParameterValue(
                limit_switch_trigger_when_below, value_type=bool),
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
            limit_switch_can_id_arg,
            limit_switch_active_high_arg,
            limit_switch_trigger_position_arg,
            limit_switch_trigger_when_below_arg,
            homing_search_direction_arg,
            homing_approach_speed_arg,
            homing_backoff_speed_arg,
            homing_fine_speed_arg,
            homing_backoff_distance_arg,
            robot_state_publisher_node,
            mock_servo_node,
            ros2_control_node,
            *spawners,
        ]
    )
