from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_int_arg(context, name):
    return int(LaunchConfiguration(name).perform(context), 0)


def _launch_node(context):
    return [
        Node(
            package="mr2_science_module",
            executable="science_module_can_node",
            name="science_module_can",
            output="screen",
            parameters=[
                {
                    "can_iface": LaunchConfiguration("can_iface"),
                    "centrifuge_module_rx_id": _parse_int_arg(
                        context, "centrifuge_module_rx_id"
                    ),
                    "carriage_module_rx_id": _parse_int_arg(
                        context, "carriage_module_rx_id"
                    ),
                    "carriage_motor_rx_id": _parse_int_arg(
                        context, "carriage_motor_rx_id"
                    ),
                    "carriage_motor_velocity_tx_id": _parse_int_arg(
                        context, "carriage_motor_velocity_tx_id"
                    ),
                    "carriage_motor_position_tx_id": _parse_int_arg(
                        context, "carriage_motor_position_tx_id"
                    ),
                    "centrifuge_motor_rx_id": _parse_int_arg(
                        context, "centrifuge_motor_rx_id"
                    ),
                    "drill_motor_rx_id": _parse_int_arg(context, "drill_motor_rx_id"),
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("can_iface", default_value="can0"),
            DeclareLaunchArgument("centrifuge_module_rx_id", default_value="0x400"),
            DeclareLaunchArgument("carriage_module_rx_id", default_value="0x500"),
            DeclareLaunchArgument("carriage_motor_rx_id", default_value="0x650"),
            DeclareLaunchArgument(
                "carriage_motor_velocity_tx_id", default_value="0x651"
            ),
            DeclareLaunchArgument(
                "carriage_motor_position_tx_id", default_value="0x652"
            ),
            DeclareLaunchArgument("centrifuge_motor_rx_id", default_value="0x600"),
            DeclareLaunchArgument("drill_motor_rx_id", default_value="0x700"),
            OpaqueFunction(function=_launch_node),
        ]
    )
