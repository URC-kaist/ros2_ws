from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mr2_led",
                executable="led_can_node",
                name="mr2_led",
                parameters=[{"can_iface": "can0", "can_id": 0x123}],
            )
        ]
    )
