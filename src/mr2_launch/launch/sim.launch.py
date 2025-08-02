from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # ─── Arguments ───────────────────────────────────────────────────────────────
    default_rviz = PathJoinSubstitution(
        [FindPackageShare("mr2_launch"), "rviz", "sim.rviz"]
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="Full path to RViz2 config file",
    )

    # ─── Nodes / Includes ────────────────────────────────────────────────────────
    pc2_to_heightmap = Node(
        package="mr2_autonomous",
        executable="pc2_to_heightmap",
        name="pc2_to_heightmap",
        output="screen",
    )

    rover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_rover_description"), "launch", "rover.launch.py"]
            )
        )
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
    )

    # ─── LaunchDescription ───────────────────────────────────────────────────────
    return LaunchDescription([rviz_arg, rover_launch, pc2_to_heightmap, rviz2])
