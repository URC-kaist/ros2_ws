from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ─── Arguments ───────────────────────────────────────────────────────────────
    default_rviz = PathJoinSubstitution([
        FindPackageShare("mr2_launch"),
        "rviz",
        "sim.rviz",
    ])
    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="Full path to RViz2 config file",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run without GUI components",
    )

    # ─── Nodes / Includes ────────────────────────────────────────────────────────
    use_sim_time = SetParameter(name="use_sim_time", value=True)

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
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless")
        }.items(),
    )

    traversibility_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("mr2_autonomous"),
                    "launch",
                    "traversibility_map.launch.py",
                ]
            )
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_moveit"), "launch", "move_group.launch.py"]
            )
        )
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )

    # ─── LaunchDescription ───────────────────────────────────────────────────────
    return LaunchDescription([
        rviz_arg,
        headless_arg,
        use_sim_time,
        rover_launch,
        pc2_to_heightmap,
        traversibility_map_launch,
        move_group_launch,
        rviz2,
    ])
