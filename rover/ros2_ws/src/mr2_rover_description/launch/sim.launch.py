import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from mr2_rover_description.launch_common import (
    controller_spawners,
    declare_can_iface,
    declare_controller_config,
    robot_description_from_xacro,
    robot_state_publisher_node,
)


def generate_launch_description():
    desc_pkg = FindPackageShare("mr2_rover_description")
    desc_share_dir = get_package_share_directory("mr2_rover_description")

    xacro_file = PathJoinSubstitution([desc_pkg, "urdf", "rover.urdf.xacro"])
    world_file = PathJoinSubstitution([desc_pkg, "worlds", "world.sdf"])

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time; normally true for Gazebo workflows",
    )

    can_iface, can_iface_arg = declare_can_iface(
        description="CAN interface used by the AK servo hardware",
    )

    controller_config, controller_config_arg = declare_controller_config(
        default_value=PathJoinSubstitution(
            [desc_pkg, "config", "controllers", "rover_controllers.yaml"]
        ),
        description="YAML file with controller manager configuration",
    )

    robot_description = robot_description_from_xacro(
        xacro_file,
        {
            "ros2_control_mode": TextSubstitution(text="gazebo"),
            "can_iface": can_iface,
            "ros2_control_config": controller_config,
        },
    )

    headless = LaunchConfiguration("headless")
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo without GUI",
    )

    # ───── Gazebo (use ros_gz_sim launcher) ──────────────────────────────
    gz_args = [
        PythonExpression(
            [
                "'-r --headless-rendering -s ' if '",
                headless,
                "' == 'true' else '-r '",
            ]
        ),
        world_file,
    ]
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    # ───── robot_state_publisher & ros2_control_node ────────────────────
    rsp = robot_state_publisher_node(robot_description, use_sim_time)

    battery_emulator_1 = Node(
        package="mr2_battery_monitor",
        executable="battery_emulator_node",
        name="battery_1",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("battery/telemetry", "battery_1/telemetry"),
            ("battery/state", "battery_1/state"),
        ],
    )

    battery_emulator_2 = Node(
        package="mr2_battery_monitor",
        executable="battery_emulator_node",
        name="battery_2",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"state_of_charge_pct": 65.0, "pack_voltage_v": 38.0},
        ],
        remappings=[
            ("battery/telemetry", "battery_2/telemetry"),
            ("battery/state", "battery_2/state"),
        ],
    )

    # ───── spawn the robot into Gazebo ───────────────────────────────────
    spawn = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=["-topic", "robot_description", "-name", "rover", "-z", "0.3"],
            )
        ],
    )

    bridge_config_path = os.path.join(
        get_package_share_directory("mr2_rover_description"),
        "config",
        "gz_bridge_topics.yaml",
    )
    with open(bridge_config_path, "r", encoding="utf-8") as bridge_config:
        bridge_topics = yaml.safe_load(bridge_config).get("topics", [])
    bridge_args = [
        f"{topic['name']}@{topic['ros_type']}@{topic['gz_type']}"
        for topic in bridge_topics
    ]

    # Make Gazebo/Ignition able to resolve package assets (meshes/textures/etc.)
    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    gz_resource_path = (
        f"{existing_gz_path}:{desc_share_dir}" if existing_gz_path else desc_share_dir
    )
    existing_ign_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    ign_resource_path = (
        f"{existing_ign_path}:{desc_share_dir}" if existing_ign_path else desc_share_dir
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_args,
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("/world/empty_world/clock", "/clock"),
            ("/imu", "/rgbd_camera/imu"),  # match RealSense IMU topic used in robot_localization
            ("/rgbd_camera/image", "/rgbd_camera/color/image_raw"),
            ("/rgbd_camera/camera_info", "/rgbd_camera/color/camera_info"),
            ("/rgbd_camera/depth_image", "/rgbd_camera/depth/image_rect_raw"),
            ("/rgbd_camera/points", "/rgbd_camera/depth/color/points"),
        ],
        output="screen",
    )

    # ───── load controllers (after ros2_control is running) ─────────────
    spawners = controller_spawners(
        ["joint_state_broadcaster", "rover_controller", "manipulator_controller"],
        start_after=2.0,
        interval=2.0,
    )

    return LaunchDescription(
        [
            headless_arg,
            use_sim_time_arg,
            can_iface_arg,
            controller_config_arg,
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", gz_resource_path),
            SetEnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", ign_resource_path),
            gz_sim,
            rsp,
            battery_emulator_1,
            battery_emulator_2,
            spawn,
            *spawners,
            gz_bridge,
        ]
    )
