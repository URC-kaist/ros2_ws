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
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="sim",
        description="Operating mode: 'sim' for Gazebo or 'real' for CAN hardware",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value=PythonExpression(
            ["'", LaunchConfiguration("mode"), "' == 'sim'"]
        ),
        description="Use simulation time; defaults to true in sim mode and false in real",
    )
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
    use_mock_servos_arg = DeclareLaunchArgument(
        "use_mock_servos",
        default_value="false",
        description="Start mock AK servo nodes instead of hardware interfaces (real mode)",
    )
    use_servo_arg = DeclareLaunchArgument(
        "use_servo",
        default_value="false",
        description="If true, launch MoveIt Servo instead of move_group",
    )
    use_sik_bridge_sim_arg = DeclareLaunchArgument(
        "use_sik_bridge_sim",
        default_value="false",
        description="If true, launch the SiK bridge node in sim mode using /dev/pts/6",
    )

    # ─── Nodes / Includes ────────────────────────────────────────────────────────
    sim_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'sim'"])
    )
    real_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'real'"])
    )
    sik_sim_condition = IfCondition(
        PythonExpression(
            [
                "'",
                LaunchConfiguration("mode"),
                "' == 'sim' and '",
                LaunchConfiguration("use_sik_bridge_sim"),
                "' == 'true'",
            ]
        )
    )
    use_sim_time_param = SetParameter(
        name="use_sim_time", value=LaunchConfiguration("use_sim_time")
    )

    pc2_to_heightmap = Node(
        package="mr2_autonomous",
        executable="pc2_to_heightmap",
        name="pc2_to_heightmap",
        output="screen",
    )

    system_status = Node(
        package="mr2_system_status",
        executable="system_status",
        name="system_status",
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
            "use_sim_time": LaunchConfiguration("use_sim_time"),
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
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_mock_servos": LaunchConfiguration("use_mock_servos"),
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
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_servo")),
    )

    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_moveit"), "launch", "realtime_servo.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_servo")),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("headless")),
    )

    sik_bridge = Node(
        package="mr2_sik_bridge",
        executable="sik_bridge_node",
        name="sik_bridge",
        output="screen",
        parameters=[
            {"device": "/dev/ttyUSB0"},
            {"baud": 57600},
            {"heartbeat_timeout_ms": 500},
        ],
        condition=real_condition,
    )
    sik_bridge_sim = Node(
        package="mr2_sik_bridge",
        executable="sik_bridge_node",
        name="sik_bridge_sim",
        output="screen",
        parameters=[
            {"device": "/dev/pts/3"},
            {"baud": 57600},
            {"heartbeat_timeout_ms": 500},
            {"log_frames": True},
        ],
        condition=sik_sim_condition,
    )

    # ─── LaunchDescription ───────────────────────────────────────────────────────
    return LaunchDescription([
        rviz_arg,
        mode_arg,
        use_sim_time_arg,
        headless_arg,
        can_iface_arg,
        controller_config_arg,
        use_mock_servos_arg,
        use_servo_arg,
        use_sik_bridge_sim_arg,
        use_sim_time_param,
        rover_launch,
        rover_real_launch,
        system_status,
        pc2_to_heightmap,
        traversibility_map_launch,
        move_group_launch,
        servo_launch,
        sik_bridge,
        sik_bridge_sim,
        rviz2,
    ])
