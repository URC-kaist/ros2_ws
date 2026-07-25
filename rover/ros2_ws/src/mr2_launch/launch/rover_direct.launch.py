"""
Run the manually operated rover stack entirely on the Jetson.

This wrapper deliberately bypasses rover_real.launch.py because that wrapper
owns the GNSS, NTRIP, localization, autonomous-navigation, and science modules.
The local XBEE PTY pair, gateway, and manual rover ROS stack are all owned by
this launch so one command brings up rover-direct operation.
"""

from pathlib import Path
import os
import shutil
import subprocess

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from mr2_launch.env import load_mr2_env


def _gateway_entrypoint():
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "base" / "gateway" / "index.js"
        if candidate.is_file():
            return str(candidate)
    raise RuntimeError(
        "Could not locate base/gateway/index.js from the mr2-stack checkout"
    )


def _node_binary():
    """Resolve and validate the repository-pinned Node.js runtime."""
    nvm_dir = Path(os.environ.get("NVM_DIR", Path.home() / ".nvm"))
    for parent in Path(__file__).resolve().parents:
        version_file = parent / ".nvmrc"
        if not version_file.is_file():
            continue
        version = version_file.read_text(encoding="utf-8").strip().removeprefix("v")
        candidate = nvm_dir / "versions" / "node" / f"v{version}" / "bin" / "node"
        if candidate.is_file():
            return str(candidate)
        raise RuntimeError(
            f"Node.js {version} is pinned by {version_file} but is not installed "
            f"at {candidate}"
        )

    candidate = shutil.which("node")
    if candidate is None:
        raise RuntimeError("Node.js 24 is required but no node executable was found")
    major = subprocess.run(
        [candidate, "-p", 'process.versions.node.split(".")[0]'],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    ).stdout.strip()
    if major != "24":
        raise RuntimeError(
            f"Node.js 24 is required; found {candidate} reporting major {major}"
        )
    return candidate


def _launch_flag_enabled(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _ensure_nginx(context):
    if not _launch_flag_enabled(context, "start_gateway"):
        return []
    if not _launch_flag_enabled(context, "ensure_nginx"):
        return []

    active = subprocess.run(
        ["systemctl", "is-active", "--quiet", "nginx.service"],
        check=False,
    )
    if active.returncode == 0:
        return [LogInfo(msg="nginx.service is active")]

    start = subprocess.run(
        ["systemctl", "--no-ask-password", "start", "nginx.service"],
        check=False,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
    )
    if start.returncode == 0:
        return [LogInfo(msg="Started nginx.service for rover-direct")]

    detail = start.stdout.strip()
    if detail:
        detail = f" systemctl reported: {detail}"
    raise RuntimeError(
        "nginx.service is inactive and rover-direct could not start it "
        "non-interactively. Run 'sudo systemctl start nginx' and retry, or use "
        "'ensure_nginx:=false' when the HTTPS dashboard is intentionally "
        f"disabled.{detail}"
    )


def generate_launch_description():
    load_mr2_env()

    can_iface_arg = DeclareLaunchArgument(
        "can_iface",
        default_value="can0",
        description="SocketCAN interface connected to the rover hardware",
    )
    controller_spawn_delay_arg = DeclareLaunchArgument(
        "controller_spawn_delay",
        default_value="10.0",
        description="Delay before spawning ros2_control controllers",
    )
    controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("mr2_rover_description"),
                "config",
                "controllers",
                "rover_controllers.yaml",
            ]
        ),
        description="Controller manager YAML for the real rover",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="true",
        description="Run without RViz",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("mr2_launch"), "rviz", "sim.rviz"]
        ),
        description="RViz configuration used when headless is false",
    )
    use_mock_servos_arg = DeclareLaunchArgument(
        "use_mock_servos",
        default_value="false",
        description="Use mock manipulator servo devices instead of CAN hardware",
    )
    enable_manipulator_module_arg = DeclareLaunchArgument(
        "enable_manipulator_module",
        default_value="true",
        description=(
            "Enable manipulator URDF, ros2_control hardware, controllers, and "
            "MoveIt Servo"
        ),
    )
    start_manipulator_controllers_active_arg = DeclareLaunchArgument(
        "start_manipulator_controllers_active",
        default_value="true",
        description=(
            "Start manipulator and gripper controllers active instead of "
            "configured/inactive"
        ),
    )
    enable_autonomous_module_arg = DeclareLaunchArgument(
        "enable_autonomous_module",
        default_value="false",
        description=(
            "Enable the RealSense/front camera and YOLO perception; direct mode "
            "does not start GNSS, localization, or Nav2"
        ),
    )
    enable_aruco_arg = DeclareLaunchArgument(
        "enable_aruco",
        default_value="false",
        description="Start the ArUco tracker",
    )
    aruco_cam_topic_arg = DeclareLaunchArgument(
        "aruco_cam_topic",
        default_value="/front_camera/image_raw",
        description="Image base topic used by the ArUco tracker",
    )
    enable_video_streaming_arg = DeclareLaunchArgument(
        "enable_video_streaming",
        default_value="true",
        description="Start the rover H.264 RTP/UDP video streaming node",
    )
    yolo_cam_topic_arg = DeclareLaunchArgument(
        "yolo_cam_topic",
        default_value="/rgbd_camera",
        description="RealSense camera base topic used by YOLO",
    )
    yolo_device_arg = DeclareLaunchArgument(
        "yolo_device",
        default_value="cuda:0",
        description="Ultralytics inference device, for example cuda:0 or cpu",
    )
    yolo_publish_annotated_arg = DeclareLaunchArgument(
        "yolo_publish_annotated",
        default_value="true",
        description="Publish annotated YOLO debug images",
    )
    yolo_annotated_fps_arg = DeclareLaunchArgument(
        "yolo_annotated_fps",
        default_value="0.5",
        description="Maximum annotated YOLO image publish rate in Hz",
    )
    enable_led_arg = DeclareLaunchArgument(
        "enable_led",
        default_value="false",
        description="Start the status and mission LED CAN nodes",
    )
    led_can_id_arg = DeclareLaunchArgument(
        "led_can_id",
        default_value="0x123",
        description="Standard CAN ID for the LED controller",
    )
    enable_camera_turret_arg = DeclareLaunchArgument(
        "enable_camera_turret",
        default_value="false",
        description="Start the camera turret CAN command node",
    )
    camera_turret_can_id_arg = DeclareLaunchArgument(
        "camera_turret_can_id",
        default_value="0x124",
        description="Standard CAN ID for the camera turret controller",
    )
    camera_turret_invert_x_arg = DeclareLaunchArgument(
        "camera_turret_invert_x",
        default_value="false",
        description="Invert camera turret X command direction",
    )
    camera_turret_invert_y_arg = DeclareLaunchArgument(
        "camera_turret_invert_y",
        default_value="false",
        description="Invert camera turret Y command direction",
    )
    xbee_device_arg = DeclareLaunchArgument(
        "xbee_device",
        default_value="/tmp/mr2_xbee_rover",
        description="Rover side of the local XBEE simulation PTY pair",
    )
    xbee_gateway_device_arg = DeclareLaunchArgument(
        "xbee_gateway_device",
        default_value="/tmp/mr2_xbee_gateway",
        description="Gateway side of the local XBEE simulation PTY pair",
    )
    start_xbee_sim_arg = DeclareLaunchArgument(
        "start_xbee_sim",
        default_value="true",
        description="Create the local PTY pair inside this launch",
    )
    start_gateway_arg = DeclareLaunchArgument(
        "start_gateway",
        default_value="true",
        description="Start the co-located rover-direct Node.js gateway",
    )
    ensure_nginx_arg = DeclareLaunchArgument(
        "ensure_nginx",
        default_value="true",
        description=(
            "Require nginx for the HTTPS dashboard and try to start its system "
            "service non-interactively when inactive"
        ),
    )
    start_rover_arg = DeclareLaunchArgument(
        "start_rover",
        default_value="true",
        description="Start the hardware-facing manual rover ROS stack",
    )
    node_binary_arg = DeclareLaunchArgument(
        "node_binary",
        default_value=_node_binary(),
        description=(
            "Node.js executable used for the rover-direct gateway; defaults to "
            "the version pinned by the repository .nvmrc"
        ),
    )
    gateway_entrypoint_arg = DeclareLaunchArgument(
        "gateway_entrypoint",
        default_value=_gateway_entrypoint(),
        description="Path to base/gateway/index.js in the mr2-stack checkout",
    )
    gateway_host_arg = DeclareLaunchArgument(
        "gateway_host",
        default_value="127.0.0.1",
        description="Gateway HTTP/WebSocket listen address",
    )
    gateway_port_arg = DeclareLaunchArgument(
        "gateway_port",
        default_value="8081",
        description="Gateway HTTP/WebSocket listen port",
    )
    video_base_host_arg = DeclareLaunchArgument(
        "video_base_host",
        default_value="127.0.0.1",
        description="Loopback destination used by the co-located video gateway",
    )
    video_config_arg = DeclareLaunchArgument(
        "video_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("mr2_launch"), "config", "video_streams.json"]
        ),
        description="Shared rover/gateway video stream configuration",
    )

    xbee_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "xbee_sim.launch.py"]
            )
        ),
        launch_arguments={
            "enable_xbee_sim": LaunchConfiguration("start_xbee_sim"),
            "xbee_sim_device": LaunchConfiguration("xbee_device"),
            "xbee_sim_peer": LaunchConfiguration("xbee_gateway_device"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_xbee_sim")),
    )

    gateway = ExecuteProcess(
        cmd=[
            LaunchConfiguration("node_binary"),
            LaunchConfiguration("gateway_entrypoint"),
            "--gateway-profile",
            "rover-direct",
            "--base-xbee-device",
            LaunchConfiguration("xbee_gateway_device"),
            "--gateway-host",
            LaunchConfiguration("gateway_host"),
            "--gateway-port",
            LaunchConfiguration("gateway_port"),
            "--video-config",
            LaunchConfiguration("video_config"),
            "--mavproxy-enable",
            "false",
            "--antenna-enable",
            "false",
            "--rocket-m2-enable",
            "false",
            "--ros-topic-relay-enable",
            "false",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_gateway")),
    )

    rover = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "rover.launch.py"]
            )
        ),
        launch_arguments={
            "mode": "real",
            "use_sim_time": "false",
            "headless": LaunchConfiguration("headless"),
            "rviz_config": LaunchConfiguration("rviz_config"),
            "can_iface": LaunchConfiguration("can_iface"),
            "controller_config": LaunchConfiguration("controller_config"),
            "controller_spawn_delay": LaunchConfiguration("controller_spawn_delay"),
            "use_mock_servos": LaunchConfiguration("use_mock_servos"),
            "enable_manipulator_module": LaunchConfiguration(
                "enable_manipulator_module"
            ),
            "enable_autonomous_module": LaunchConfiguration(
                "enable_autonomous_module"
            ),
            "enable_localization": "false",
            "start_manipulator_controllers_active": LaunchConfiguration(
                "start_manipulator_controllers_active"
            ),
            "enable_aruco": LaunchConfiguration("enable_aruco"),
            "aruco_cam_topic": LaunchConfiguration("aruco_cam_topic"),
            "xbee_device": LaunchConfiguration("xbee_device"),
            "enable_video_streaming": LaunchConfiguration(
                "enable_video_streaming"
            ),
            "video_base_host": LaunchConfiguration("video_base_host"),
            "video_config": LaunchConfiguration("video_config"),
            "yolo_cam_topic": LaunchConfiguration("yolo_cam_topic"),
            "yolo_device": LaunchConfiguration("yolo_device"),
            "yolo_publish_annotated": LaunchConfiguration(
                "yolo_publish_annotated"
            ),
            "yolo_annotated_fps": LaunchConfiguration("yolo_annotated_fps"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_rover")),
    )

    led_node = Node(
        package="mr2_led",
        executable="led_can_node",
        name="mr2_led",
        output="screen",
        parameters=[
            {"can_iface": LaunchConfiguration("can_iface")},
            {"can_id": LaunchConfiguration("led_can_id")},
        ],
        condition=IfCondition(LaunchConfiguration("enable_led")),
    )
    mission_status_led_node = Node(
        package="mr2_led",
        executable="mission_status_led_node",
        name="mr2_mission_status_led",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_led")),
    )
    camera_turret = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("mr2_camera_turret"),
                    "launch",
                    "camera_turret_can.launch.py",
                ]
            )
        ),
        launch_arguments={
            "can_iface": LaunchConfiguration("can_iface"),
            "can_id": LaunchConfiguration("camera_turret_can_id"),
            "invert_x": LaunchConfiguration("camera_turret_invert_x"),
            "invert_y": LaunchConfiguration("camera_turret_invert_y"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_camera_turret")),
    )
    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "realsense_rgbd.launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": "rgbd_camera",
            "camera_namespace": "",
            "base_frame_id": "rgbd_camera",
            "urdf_mount_frame": "rgbd_camera",
            "imu_only": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_autonomous_module")),
    )
    front_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "front_uvc_camera.launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": "front_camera",
            "camera_namespace": "front_camera",
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_autonomous_module")),
    )
    rover_extras = GroupAction(
        actions=[
            led_node,
            mission_status_led_node,
            camera_turret,
            realsense,
            front_camera,
        ],
        condition=IfCondition(LaunchConfiguration("start_rover")),
    )

    # Give socat time to create both symlinks before the rover bridge opens its
    # endpoint. The gateway reconnects automatically if it wins the startup race.
    delayed_rover = TimerAction(period=1.0, actions=[rover])

    return LaunchDescription(
        [
            can_iface_arg,
            controller_spawn_delay_arg,
            controller_config_arg,
            headless_arg,
            rviz_config_arg,
            use_mock_servos_arg,
            enable_manipulator_module_arg,
            start_manipulator_controllers_active_arg,
            enable_autonomous_module_arg,
            enable_aruco_arg,
            aruco_cam_topic_arg,
            enable_video_streaming_arg,
            yolo_cam_topic_arg,
            yolo_device_arg,
            yolo_publish_annotated_arg,
            yolo_annotated_fps_arg,
            enable_led_arg,
            led_can_id_arg,
            enable_camera_turret_arg,
            camera_turret_can_id_arg,
            camera_turret_invert_x_arg,
            camera_turret_invert_y_arg,
            xbee_device_arg,
            xbee_gateway_device_arg,
            start_xbee_sim_arg,
            start_gateway_arg,
            ensure_nginx_arg,
            start_rover_arg,
            node_binary_arg,
            gateway_entrypoint_arg,
            gateway_host_arg,
            gateway_port_arg,
            video_base_host_arg,
            video_config_arg,
            OpaqueFunction(function=_ensure_nginx),
            xbee_sim,
            gateway,
            delayed_rover,
            rover_extras,
        ]
    )
