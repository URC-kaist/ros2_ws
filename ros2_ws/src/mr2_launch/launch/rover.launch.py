from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
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
    foxglove_port_arg = DeclareLaunchArgument(
        "foxglove_port",
        default_value="8765",
        description="WebSocket port for the Foxglove Bridge",
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
    sik_sim_device_arg = DeclareLaunchArgument(
        "sik_sim_device",
        default_value="/tmp/sik_sim0",
        description="Path to the PTY device that the SiK bridge will open in sim mode",
    )
    sik_sim_peer_arg = DeclareLaunchArgument(
        "sik_sim_peer",
        default_value="/tmp/sik_sim1",
        description="Path to the peer PTY that external tools can attach to",
    )
    sik_sim_baud_arg = DeclareLaunchArgument(
        "sik_sim_baud",
        default_value="57600",
        description="Baud rate for the simulated SiK link",
    )
    aruco_cam_topic_arg = DeclareLaunchArgument(
        "aruco_cam_topic",
        default_value="/rgbd_camera/image",
        description="Base image topic for aruco_opencv (must have matching /camera_info; default is Gazebo RGBD camera)",
    )

    # ─── Nodes / Includes ────────────────────────────────────────────────────────
    sim_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'sim'"])
    )
    real_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'real'"])
    )
    sik_sim_condition = sim_condition
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

    sik_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "sik_sim.launch.py"]
            )
        ),
        launch_arguments={
            "sik_sim_device": LaunchConfiguration("sik_sim_device"),
            "sik_sim_peer": LaunchConfiguration("sik_sim_peer"),
        }.items(),
        condition=sik_sim_condition,
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

    localization_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("mr2_rover_auto"), "launch", "localization.launch.py"]
                    )
                ),
                launch_arguments={
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            )
        ],
    )

    traversability_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("mr2_rover_auto"),
                #   FindPackageShare("mr2_autonomous"),
                    "launch",
                    "traversability_pipeline.launch.py",
                #   "traversability_map.launch.py",
                ]
            )
        )
    )

    aruco_tracker = Node(
        package="aruco_opencv",
        executable="aruco_tracker_autostart",
        name="aruco_tracker",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("aruco_opencv"), "config", "aruco_tracker.yaml"]
            ),
            {
                "board_descriptions_path": PathJoinSubstitution(
                    [
                        FindPackageShare("mr2_launch"),
                        "config",
                        "board_descriptions.yaml",
                    ]
                ),
                "cam_base_topic": LaunchConfiguration("aruco_cam_topic"),
                "marker_size": 0.20,  # 20 cm face as observed on the post
                "image_is_rectified": False,
                "aruco.detectInvertedMarker": True,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
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
            {"device": LaunchConfiguration("sik_sim_device")},
            {"baud": LaunchConfiguration("sik_sim_baud")},
            {"heartbeat_timeout_ms": 500},
            {"log_frames": False},
        ],
        condition=sik_sim_condition,
    )

    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("foxglove_bridge"),
                    "launch",
                    "foxglove_bridge_launch.xml",
                ]
            )
        ),
        launch_arguments={
            "port": LaunchConfiguration("foxglove_port"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    rosbridge_ws = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[
            {"port": 9090},
            {"default_call_service_timeout": 0.0},
            {"call_services_in_new_thread": False},
            {"send_action_goals_in_new_thread": False},
        ],
    )

    # ─── LaunchDescription ───────────────────────────────────────────────────────
    return LaunchDescription([
        rviz_arg,
        mode_arg,
        use_sim_time_arg,
        headless_arg,
        foxglove_port_arg,
        can_iface_arg,
        controller_config_arg,
        use_mock_servos_arg,
        use_servo_arg,
        sik_sim_device_arg,
        sik_sim_peer_arg,
        sik_sim_baud_arg,
        aruco_cam_topic_arg,
        use_sim_time_param,
        rover_launch,
        rover_real_launch,
        localization_launch,
        system_status,
        # pc2_to_heightmap,
        traversability_map_launch,
        aruco_tracker,
        move_group_launch,
        servo_launch,
        sik_sim_launch,
        sik_bridge,
        sik_bridge_sim,
        foxglove_bridge,
        rosbridge_ws,
        rviz2,
    ])
