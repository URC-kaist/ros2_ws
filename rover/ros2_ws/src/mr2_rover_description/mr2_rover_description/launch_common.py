from numbers import Real
from typing import Iterable, List, Mapping, Sequence

from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def declare_can_iface(default_value: str = "can0", description: str = ""):
    """Return a LaunchConfiguration and matching DeclareLaunchArgument for the CAN iface."""
    can_iface = LaunchConfiguration("can_iface")
    can_iface_arg = DeclareLaunchArgument(
        "can_iface",
        default_value=default_value,
        description=description or "CAN interface used by AK servo hardware",
    )
    return can_iface, can_iface_arg


def declare_controller_config(default_value, description: str = ""):
    """Return a LaunchConfiguration and argument for controller manager YAML path."""
    controller_config = LaunchConfiguration("controller_config")
    controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=default_value,
        description=description or "YAML file containing ros2_control controller manager config",
    )
    return controller_config, controller_config_arg


def declare_use_mock_servos(default: str = "false", description: str = ""):
    use_mock_servos = LaunchConfiguration("use_mock_servos")
    use_mock_servos_arg = DeclareLaunchArgument(
        "use_mock_servos",
        default_value=default,
        description=description or "Start mock AK servo nodes instead of hardware interfaces",
    )
    return use_mock_servos, use_mock_servos_arg


def robot_description_from_xacro(xacro_file, mappings: Mapping[str, LaunchConfiguration]):
    """Generate the robot_description mapping from a xacro path and substitution map."""
    cmd_parts: List[object] = ["xacro ", xacro_file]
    for key, value in mappings.items():
        cmd_parts.extend([" ", key, ":=", value])
    # Wrap Command in ParameterValue to keep it a plain string (avoid YAML parsing errors)
    return {"robot_description": ParameterValue(Command(cmd_parts), value_type=str)}


def robot_state_publisher_node(robot_description, use_sim_time):
    """Standard robot_state_publisher node with shared sim time handling."""
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": use_sim_time}],
        output="screen",
    )


def ros2_control_node(controller_config, robot_description, use_sim_time, remappings=None):
    """ros2_control_node with consistent parameters and optional remappings."""
    params = [controller_config, robot_description, {"use_sim_time": use_sim_time}]
    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=params,
        remappings=remappings or [],
        output="screen",
    )


def controller_spawners(
    controller_names: Sequence[str],
    start_after: float = 2.0,
    interval: float = 1.0,
    output: str = "screen",
    inactive_controllers: Sequence[str] = (),
):
    """Create staggered controller spawner TimerActions for a list of controllers."""
    actions = []
    for index, name in enumerate(controller_names):
        if isinstance(start_after, Real):
            delay = float(start_after) + (index * interval)
        else:
            delay = PythonExpression([start_after, f" + {index * interval}"])
        args = [name]
        if name in inactive_controllers:
            args.append("--inactive")
        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=args,
                        output=output,
                    )
                ],
            )
        )
    return actions


def mock_servo_nodes(
    can_iface,
    motor_ids: Iterable[int],
    condition,
    *,
    output: str = "screen",
    enable_limit_switch: bool = False,
    enable_absolute_encoder: bool = False,
):
    """Generate mock AK servo nodes with minimal shared configuration."""
    nodes = []
    for motor_id in motor_ids:
        params = {
            "can_iface": can_iface,
            "motor_id": ParameterValue(motor_id, value_type=int),
        }
        if enable_limit_switch:
            params["limit_switch_enabled"] = True
        if enable_absolute_encoder:
            params["absolute_encoder_enabled"] = True

        nodes.append(
            Node(
                package="mr2_devices_ak_servo",
                executable="mock_ak_servo_node",
                parameters=[params],
                condition=condition,
                output=output,
            )
        )
    return nodes
