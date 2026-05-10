import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def _load_yaml(package_name: str, relative_path: str):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, relative_path), "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time when running against simulation/backfilled data.",
        ),
    ]

    moveit_config = MoveItConfigsBuilder("rover", package_name="mr2_moveit").to_moveit_configs()
    moveit_params = moveit_config.to_dict()
    for key in (
        "moveit_controller_manager",
        "moveit_manage_controllers",
        "moveit_simple_controller_manager",
    ):
        moveit_params.pop(key, None)

    servo_dict = _load_yaml("mr2_moveit", "config/servo.yaml")
    servo_params = {"moveit_servo": servo_dict}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="moveit_servo",
        output="screen",
        parameters=[
            servo_params,
            moveit_params,
            {"use_sim_time": use_sim_time},
        ],
    )

    start_servo = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/moveit_servo/start_servo",
                    "std_srvs/srv/Trigger",
                    "{}",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(args + [servo_node, start_servo])
