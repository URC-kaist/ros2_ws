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
from launch_ros.parameter_descriptions import ParameterValue
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
            ["'true' if '", LaunchConfiguration("mode"), "' == 'sim' else 'false'"]
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
    enable_manipulator_arg = DeclareLaunchArgument(
        "enable_manipulator",
        default_value="true",
        description="Enable manipulator URDF, ros2_control, and MoveIt2 components",
    )
    enable_manipulator_sim_arg = DeclareLaunchArgument(
        "enable_manipulator_sim",
        default_value="false",
        description="Enable manipulator URDF and ros2_control in simulation",
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
    sik_device_arg = DeclareLaunchArgument(
        "sik_device",
        default_value="/dev/ttyUSB0",
        description="Serial device for the real SiK bridge",
    )
    sik_baud_arg = DeclareLaunchArgument(
        "sik_baud",
        default_value="57600",
        description="Baud rate for the real SiK bridge",
    )
    enable_aruco_arg = DeclareLaunchArgument(
        "enable_aruco",
        default_value="true",
        description="Start ArUco tracker node",
    )
    aruco_cam_topic_arg = DeclareLaunchArgument(
        "aruco_cam_topic",
        default_value="/front_camera/image_raw",
        description="Base image topic for aruco_opencv (must have matching /camera_info; default is Gazebo RGBD camera)",
    )
    enable_yolo_arg = DeclareLaunchArgument(
        "enable_yolo",
        default_value="true",
        description="Start YOLO RGBD detector node",
    )
    yolo_cam_topic_arg = DeclareLaunchArgument(
        "yolo_cam_topic",
        default_value="/rgbd_camera",
        description="RealSense camera base topic for YOLO (e.g., /rgbd_camera)",
    )

    # ─── Nodes / Includes ────────────────────────────────────────────────────────
    sim_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'sim'"])
    )
    real_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'real'"])
    )
    enable_manipulator = LaunchConfiguration("enable_manipulator")
    sik_sim_condition = sim_condition
    use_sim_time_param = SetParameter(
        name="use_sim_time", value=LaunchConfiguration("use_sim_time")
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
            "enable_manipulator": LaunchConfiguration("enable_manipulator_sim"),
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
            "enable_manipulator": enable_manipulator,
        }.items(),
        condition=real_condition,
    )

    localization_delay = PythonExpression(
        ["'30.0' if '", LaunchConfiguration("mode"), "' == 'real' else '3.0'"]
    )
    localization_launch = TimerAction(
        period=localization_delay,
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

    aruco_tracker = Node(
        package="aruco_opencv",
        executable="aruco_tracker_autostart",
        name="aruco_tracker",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_aruco")),
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("aruco_opencv"), "config", "aruco_tracker.yaml"]
            ),
            {
                "cam_base_topic": LaunchConfiguration("aruco_cam_topic"),
                "marker_size": 0.15,
                "image_is_rectified": False,
                "aruco.detectInvertedMarker": True,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    yolo_rgb_topic = PythonExpression(
        ["'", LaunchConfiguration("yolo_cam_topic"), "/color/image_raw'"]
    )
    yolo_depth_topic = PythonExpression(
        ["'", LaunchConfiguration("yolo_cam_topic"), "/depth/image_rect_raw'"]
    )
    yolo_camera_info_topic = PythonExpression(
        ["'", LaunchConfiguration("yolo_cam_topic"), "/color/camera_info'"]
    )

    yolo_detector = Node(
        package="mr2_yolo_perception",
        executable="yolo_rgbd_detector",
        name="yolo_detector",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_yolo")),
        parameters=[
            {
                "rgb_topic": yolo_rgb_topic,
                "depth_topic": yolo_depth_topic,
                "camera_info_topic": yolo_camera_info_topic,
                "annotated_topic": "yolo/annotated_image",
                "pose_topic": "yolo/object_pose",
                "camera_frame_is_optical": False,
                "class_id_map": "0:2,1:0,2:1",
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
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
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    enable_manipulator,
                    "' == 'true' and '",
                    LaunchConfiguration("use_servo"),
                    "' != 'true'",
                ]
            )
        ),
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
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    enable_manipulator,
                    "' == 'true' and '",
                    LaunchConfiguration("use_servo"),
                    "' == 'true'",
                ]
            )
        ),
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
            {"device": LaunchConfiguration("sik_device")},
            {"baud": LaunchConfiguration("sik_baud")},
            {"heartbeat_timeout_ms": 500},
            {"cmd_vel_topic": "/base/cmd_vel"},
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
            {"cmd_vel_topic": "/base/cmd_vel"},
        ],
        condition=sik_sim_condition,
    )

    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {"port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int)},
            {"debug": ParameterValue(False, value_type=bool)},
            {"address": "0.0.0.0"},
            {"tls": ParameterValue(False, value_type=bool)},
            {"certfile": ""},
            {"keyfile": ""},
            {"topic_whitelist": [".*"]},
            {"param_whitelist": [".*"]},
            {"service_whitelist": [".*"]},
            {"client_topic_whitelist": [".*"]},
            {"min_qos_depth": 1},
            {"max_qos_depth": 10},
            {"num_threads": 0},
            {"send_buffer_limit": 10000000},
            {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
            {
                "capabilities": [
                    "clientPublish",
                    "parameters",
                    "parametersSubscribe",
                    "services",
                    "connectionGraph",
                    "assets",
                ]
            },
            {"include_hidden": ParameterValue(False, value_type=bool)},
            {
                "asset_uri_allowlist": [
                    "^package://(?:[-\\w%]+/)*[-\\w%.]+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$"
                ]
            },
            {"ignore_unresponsive_param_nodes": ParameterValue(True, value_type=bool)},
        ],
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
        enable_manipulator_arg,
        enable_manipulator_sim_arg,
        use_servo_arg,
        sik_sim_device_arg,
        sik_sim_peer_arg,
        sik_sim_baud_arg,
        sik_device_arg,
        sik_baud_arg,
        enable_aruco_arg,
        aruco_cam_topic_arg,
        enable_yolo_arg,
        yolo_cam_topic_arg,
        use_sim_time_param,
        rover_launch,
        rover_real_launch,
        localization_launch,
        system_status,
        aruco_tracker,
        yolo_detector,
        move_group_launch,
        servo_launch,
        sik_sim_launch,
        sik_bridge,
        sik_bridge_sim,
        foxglove_bridge,
        rosbridge_ws,
        rviz2,
    ])
