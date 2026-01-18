from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("mr2_launch"), "rviz", "sim.rviz"]
        ),
        description="Full path to RViz2 config file",
    )
    controller_config_arg = DeclareLaunchArgument(
        "controller_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("mr2_rover_description"), "config", "controllers", "rover_controllers.yaml"]
        ),
        description="Controller manager YAML shared by sim and hardware",
    )
    can_iface_arg = DeclareLaunchArgument(
        "can_iface",
        default_value="can0",
        description="CAN interface connected to the rover hardware",
    )
    use_mock_servos_arg = DeclareLaunchArgument(
        "use_mock_servos",
        default_value="false",
        description="Start mock AK servo nodes that emulate the manipulator CAN motors",
    )
    enable_sik_sim_arg = DeclareLaunchArgument(
        "enable_sik_sim",
        default_value="false",
        description="Start socat to emulate a SiK serial port pair in real mode",
    )
    sik_sim_device_arg = DeclareLaunchArgument(
        "sik_sim_device",
        default_value="/tmp/sik_sim0",
        description="PTy path the SiK bridge will open when socat emulation is enabled",
    )
    sik_sim_peer_arg = DeclareLaunchArgument(
        "sik_sim_peer",
        default_value="/tmp/sik_sim1",
        description="Peer PTY path for external attachment when socat emulation is enabled",
    )
    sik_sim_baud_arg = DeclareLaunchArgument(
        "sik_sim_baud",
        default_value="57600",
        description="Baud rate for the emulated SiK link",
    )
    sik_device_arg = DeclareLaunchArgument(
        "sik_device",
        default_value=PythonExpression(
            [
                "'",
                LaunchConfiguration("sik_sim_device"),
                "' if '",
                LaunchConfiguration("enable_sik_sim"),
                "' == 'true' else '/dev/ttyUSB0'",
            ]
        ),
        description="Serial device for the SiK bridge (defaults to emulated PTY when enabled)",
    )
    sik_baud_arg = DeclareLaunchArgument(
        "sik_baud",
        default_value=PythonExpression(
            [
                "'",
                LaunchConfiguration("sik_sim_baud"),
                "' if '",
                LaunchConfiguration("enable_sik_sim"),
                "' == 'true' else '57600'",
            ]
        ),
        description="Baud rate for the SiK bridge (defaults to emulated baud when enabled)",
    )
    left_gnss_serial_arg = DeclareLaunchArgument(
        "left_gnss_serial",
        default_value="TowerRx_",
        description="USB serial string for the left F9P (empty selects first match)",
    )
    right_gnss_serial_arg = DeclareLaunchArgument(
        "right_gnss_serial",
        default_value="NorthRx_",
        description="USB serial string for the right F9P (empty selects first match)",
    )
    left_gnss_frame_arg = DeclareLaunchArgument(
        "left_gnss_frame_id",
        default_value="left_gnss",
        description="Frame ID for left GNSS NavSatFix",
    )
    right_gnss_frame_arg = DeclareLaunchArgument(
        "right_gnss_frame_id",
        default_value="right_gnss",
        description="Frame ID for right GNSS NavSatFix",
    )

    sik_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "sik_sim.launch.py"]
            )
        ),
        launch_arguments={
            "enable_sik_sim": LaunchConfiguration("enable_sik_sim"),
            "sik_sim_device": LaunchConfiguration("sik_sim_device"),
            "sik_sim_peer": LaunchConfiguration("sik_sim_peer"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_sik_sim")),
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "realsense_rgbd.launch.py"]
            )
        ),
    )

    traversability_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_rover_auto"), "launch", "traversability_pipeline.launch.py"]
            )
        ),
        launch_arguments={
            "use_sim_time": "false",
        }.items(),
    )

    rover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "rover.launch.py"]
            )
        ),
        launch_arguments={
            "mode": "real",
            "rviz_config": LaunchConfiguration("rviz_config"),
            "controller_config": LaunchConfiguration("controller_config"),
            "can_iface": LaunchConfiguration("can_iface"),
            "use_mock_servos": LaunchConfiguration("use_mock_servos"),
            "sik_device": LaunchConfiguration("sik_device"),
            "sik_baud": LaunchConfiguration("sik_baud"),
            "sik_sim_device": LaunchConfiguration("sik_sim_device"),
            "sik_sim_peer": LaunchConfiguration("sik_sim_peer"),
            "sik_sim_baud": LaunchConfiguration("sik_sim_baud"),
        }.items(),
    )

    ublox_left_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "ublox_fb_r_rover_named.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": "left_gnss",
            "device_family": "F9P",
            "device_serial_string": LaunchConfiguration("left_gnss_serial"),
            "frame_id": LaunchConfiguration("left_gnss_frame_id"),
        }.items(),
    )

    ublox_right_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("mr2_launch"), "launch", "ublox_fb_r_rover_named.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": "right_gnss",
            "device_family": "F9P",
            "device_serial_string": LaunchConfiguration("right_gnss_serial"),
            "frame_id": LaunchConfiguration("right_gnss_frame_id"),
        }.items(),
    )

    left_navsat_relay = Node(
        package="topic_tools",
        executable="relay",
        name="left_gnss_navsat_relay",
        output="screen",
        arguments=["/left_gnss/fix", "/left_gnss/navsat"],
    )

    right_navsat_relay = Node(
        package="topic_tools",
        executable="relay",
        name="right_gnss_navsat_relay",
        output="screen",
        arguments=["/right_gnss/fix", "/right_gnss/navsat"],
    )

    return LaunchDescription(
        [
            rviz_arg,
            controller_config_arg,
            can_iface_arg,
            use_mock_servos_arg,
            enable_sik_sim_arg,
            sik_sim_device_arg,
            sik_sim_peer_arg,
            sik_sim_baud_arg,
            sik_device_arg,
            sik_baud_arg,
            left_gnss_serial_arg,
            right_gnss_serial_arg,
            left_gnss_frame_arg,
            right_gnss_frame_arg,
            sik_sim_launch,
            realsense_launch,
            traversability_launch,
            rover_launch,
            ublox_left_launch,
            ublox_right_launch,
            left_navsat_relay,
            right_navsat_relay,
        ]
    )
