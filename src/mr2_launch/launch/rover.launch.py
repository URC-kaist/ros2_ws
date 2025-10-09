from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
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
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="sim",
        description="Operating mode: 'sim' for Gazebo or 'real' for CAN hardware",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run without GUI components",
    )
    can_iface_arg = DeclareLaunchArgument(
        "can_iface",
        default_value="can0",
        description="CAN interface used by the AK servo hardware",
    )
    controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("mr2_rover_description"), "config", "controllers", "rover_controllers.yaml"]
        ),
        description="Controller manager YAML shared by sim and hardware",
    )

    # ─── Nodes / Includes ────────────────────────────────────────────────────────
    sim_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'sim'"])
    )
    real_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'real'"])
    )

    use_sim_time_true = SetParameter(
        name="use_sim_time", value=True, condition=sim_condition
    )
    use_sim_time_false = SetParameter(
        name="use_sim_time", value=False, condition=real_condition
    )

    pc2_to_heightmap = Node(
        package="mr2_autonomous",
        executable="pc2_to_heightmap",
        name="pc2_to_heightmap",
        output="screen",
    )

    rover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_rover_description"), "launch", "sim.launch.py"]
            )
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "can_iface": LaunchConfiguration("can_iface"),
            "controller_config": LaunchConfiguration("controller_config"),
        }.items(),
        condition=sim_condition,
    )

    rover_real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_rover_description"), "launch", "real.launch.py"]
            )
        ),
        launch_arguments={
            "can_iface": LaunchConfiguration("can_iface"),
            "controller_config": LaunchConfiguration("controller_config"),
        }.items(),
        condition=real_condition,
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
        mode_arg,
        headless_arg,
        can_iface_arg,
        controller_config_arg,
        use_sim_time_true,
        use_sim_time_false,
        rover_launch,
        rover_real_launch,
        pc2_to_heightmap,
        traversibility_map_launch,
        move_group_launch,
        rviz2,
    ])
