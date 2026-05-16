from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from mr2_launch.env import load_mr2_env


def generate_launch_description():
    load_mr2_env(required=["MR2_BASE_IP"])

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
    controller_spawn_delay_arg = DeclareLaunchArgument(
        "controller_spawn_delay",
        default_value="2.0",
        description="Delay (seconds) before spawning ros2_control controllers in real mode",
    )
    use_mock_servos_arg = DeclareLaunchArgument(
        "use_mock_servos",
        default_value="false",
        description="Start mock AK servo nodes instead of hardware interfaces (real mode)",
    )
    enable_manipulator_module_arg = DeclareLaunchArgument(
        "enable_manipulator_module",
        default_value="true",
        description="Enable manipulator URDF, ros2_control, and MoveIt2 components",
    )
    enable_manipulator_module_sim_arg = DeclareLaunchArgument(
        "enable_manipulator_module_sim",
        default_value="false",
        description="Enable manipulator URDF and ros2_control in simulation",
    )
    enable_autonomous_module_arg = DeclareLaunchArgument(
        "enable_autonomous_module",
        default_value="true",
        description="Enable autonomous camera module (front_camera)",
    )
    enable_autonomous_module_sim_arg = DeclareLaunchArgument(
        "enable_autonomous_module_sim",
        default_value="true",
        description="Enable autonomous camera module (front_camera) in simulation",
    )
    xbee_sim_device_arg = DeclareLaunchArgument(
        "xbee_sim_device",
        default_value="/tmp/xbee_sim0",
        description="Path to the PTY device that the XBEE bridge will open in sim mode",
    )
    xbee_sim_peer_arg = DeclareLaunchArgument(
        "xbee_sim_peer",
        default_value="/tmp/xbee_sim1",
        description="Path to the peer PTY that external tools can attach to",
    )
    xbee_device_arg = DeclareLaunchArgument(
        "xbee_device",
        default_value="/dev/ttyXBEE",
        description="Serial device for the real XBEE bridge",
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
    enable_video_streaming_arg = DeclareLaunchArgument(
        "enable_video_streaming",
        default_value="false",
        description="Start the rover H.264 RTP/UDP video streaming node",
    )
    video_base_host_arg = DeclareLaunchArgument(
        "video_base_host",
        default_value=EnvironmentVariable("MR2_BASE_IP"),
        description="Base-station host/IP for rover RTP/UDP video streams",
    )
    video_config_arg = DeclareLaunchArgument(
        "video_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("mr2_launch"), "config", "video_streams.json"]
        ),
        description="Central JSON video stream configuration file",
    )
    yolo_cam_topic_arg = DeclareLaunchArgument(
        "yolo_cam_topic",
        default_value="/rgbd_camera",
        description="RealSense camera base topic for YOLO (e.g., /rgbd_camera)",
    )
    yolo_device_arg = DeclareLaunchArgument(
        "yolo_device",
        default_value="",
        description="Ultralytics device for YOLO inference (e.g., cuda:0 or cpu; empty lets Ultralytics choose)",
    )
    yolo_publish_annotated_arg = DeclareLaunchArgument(
        "yolo_publish_annotated",
        default_value="true",
        description="Publish annotated YOLO debug images",
    )
    yolo_annotated_fps_arg = DeclareLaunchArgument(
        "yolo_annotated_fps",
        default_value="0.0",
        description="Maximum annotated YOLO debug image publish rate in Hz; 0 publishes every frame",
    )

    # ─── Nodes / Includes ────────────────────────────────────────────────────────
    sim_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'sim'"])
    )
    real_condition = IfCondition(
        PythonExpression(["'", LaunchConfiguration("mode"), "' == 'real'"])
    )
    enable_manipulator_module = LaunchConfiguration("enable_manipulator_module")
    xbee_sim_condition = sim_condition
    use_sim_time_param = SetParameter(
        name="use_sim_time", value=LaunchConfiguration("use_sim_time")
    )

    system_status = Node(
        package="mr2_system_status",
        executable="system_status",
        name="system_status",
        output="screen",
    )

    xbee_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "xbee_sim.launch.py"]
            )
        ),
        launch_arguments={
            "xbee_sim_device": LaunchConfiguration("xbee_sim_device"),
            "xbee_sim_peer": LaunchConfiguration("xbee_sim_peer"),
        }.items(),
        condition=xbee_sim_condition,
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
            "enable_manipulator_module": LaunchConfiguration("enable_manipulator_module_sim"),
            "enable_autonomous_module": LaunchConfiguration("enable_autonomous_module_sim"),
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
            "controller_spawn_delay": LaunchConfiguration("controller_spawn_delay"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "use_mock_servos": LaunchConfiguration("use_mock_servos"),
            "enable_manipulator_module": enable_manipulator_module,
            "enable_autonomous_module": LaunchConfiguration("enable_autonomous_module"),
        }.items(),
        condition=real_condition,
    )
    disabled_video_stream_ids = PythonExpression(
        [
            "','.join(filter(None, ["
            "'rgbd_camera' if (('",
            LaunchConfiguration("mode"),
            "' == 'real' and '",
            LaunchConfiguration("enable_autonomous_module"),
            "' == 'true') or ('",
            LaunchConfiguration("mode"),
            "' == 'sim' and '",
            LaunchConfiguration("enable_autonomous_module_sim"),
            "' == 'true')) else '', "
            "'arm_cam,gripper_cam' if (('",
            LaunchConfiguration("mode"),
            "' == 'real' and '",
            LaunchConfiguration("enable_manipulator_module"),
            "' != 'true') or ('",
            LaunchConfiguration("mode"),
            "' == 'sim' and '",
            LaunchConfiguration("enable_manipulator_module_sim"),
            "' != 'true')) else '']))",
        ]
    )
    video_streaming_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "video_streaming.launch.py"]
            )
        ),
        launch_arguments={
            "video_config": LaunchConfiguration("video_config"),
            "video_base_host": LaunchConfiguration("video_base_host"),
            "disabled_stream_ids": disabled_video_stream_ids,
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_video_streaming")),
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
        ["'", LaunchConfiguration("yolo_cam_topic"), "/aligned_depth_to_color/image_raw'"]
    )
    yolo_camera_info_topic = PythonExpression(
        ["'", LaunchConfiguration("yolo_cam_topic"), "/color/camera_info'"]
    )
    camera_frame_is_optical = PythonExpression(
        ["'true' if '", LaunchConfiguration("mode"), "' == 'real' else 'false'"]
    )

    yolo_detector = Node(
        package="mr2_yolo_perception",
        executable="yolo_rgbd_detector",
        name="yolo_detector",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_autonomous_module")),
        parameters=[
            {
                "rgb_topic": yolo_rgb_topic,
                "depth_topic": yolo_depth_topic,
                "camera_info_topic": yolo_camera_info_topic,
                "annotated_topic": "yolo/annotated_image",
                "publish_annotated": LaunchConfiguration("yolo_publish_annotated"),
                "annotated_fps": LaunchConfiguration("yolo_annotated_fps"),
                "pose_topic": "yolo/object_pose",
                "camera_frame_is_optical": camera_frame_is_optical,
                "class_id_map": "0:2,1:0,2:1",
                "device": LaunchConfiguration("yolo_device"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
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
            PythonExpression(["'", enable_manipulator_module, "' == 'true'"])
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

    xbee_bridge = Node(
        package="mr2_xbee_bridge",
        executable="xbee_bridge_node",
        name="xbee_bridge",
        output="screen",
        parameters=[
            {"device": LaunchConfiguration("xbee_device")},
            {"heartbeat_timeout_ms": 500},
            {"cmd_vel_topic": "/base/cmd_vel"},
            {"smooth_arm_joint_commands": True},
            {"arm_joint_accel_limit_rad_s2": 3.0},
        ],
        condition=real_condition,
    )
    xbee_bridge_sim = Node(
        package="mr2_xbee_bridge",
        executable="xbee_bridge_node",
        name="xbee_bridge_sim",
        output="screen",
        parameters=[
            {"device": LaunchConfiguration("xbee_sim_device")},
            {"heartbeat_timeout_ms": 500},
            {"log_frames": False},
            {"cmd_vel_topic": "/base/cmd_vel"},
            {"smooth_arm_joint_commands": True},
            {"arm_joint_accel_limit_rad_s2": 3.0},
        ],
        condition=xbee_sim_condition,
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
        can_iface_arg,
        controller_config_arg,
        controller_spawn_delay_arg,
        use_mock_servos_arg,
        enable_manipulator_module_arg,
        enable_manipulator_module_sim_arg,
        enable_autonomous_module_arg,
        enable_autonomous_module_sim_arg,
        xbee_sim_device_arg,
        xbee_sim_peer_arg,
        xbee_device_arg,
        enable_aruco_arg,
        aruco_cam_topic_arg,
        enable_video_streaming_arg,
        video_base_host_arg,
        video_config_arg,
        yolo_cam_topic_arg,
        yolo_device_arg,
        yolo_publish_annotated_arg,
        yolo_annotated_fps_arg,
        use_sim_time_param,
        rover_launch,
        rover_real_launch,
        localization_launch,
        system_status,
        aruco_tracker,
        yolo_detector,
        servo_launch,
        xbee_sim_launch,
        xbee_bridge,
        xbee_bridge_sim,
        video_streaming_launch,
        rosbridge_ws,
        rviz2,
    ])
