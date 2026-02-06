from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Optional manual datum setter for navsat_transform if base survey-in message
    # is not available on the rover network. Disabled by default.
    set_manual_datum_arg = DeclareLaunchArgument(
        "set_manual_datum",
        default_value="false",
        description="When true, call navsat_transform set_datum with provided lat/lon/alt",
    )
    datum_lat_arg = DeclareLaunchArgument(
        "datum_lat",
        default_value="0.0",
        description="WGS84 latitude for manual datum",
    )
    datum_lon_arg = DeclareLaunchArgument(
        "datum_lon",
        default_value="0.0",
        description="WGS84 longitude for manual datum",
    )
    datum_alt_arg = DeclareLaunchArgument(
        "datum_alt",
        default_value="0.0",
        description="WGS84 altitude (meters) for manual datum",
    )

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
    enable_manipulator_arg = DeclareLaunchArgument(
        "enable_manipulator",
        default_value="true",
        description="Enable manipulator URDF, ros2_control, and MoveIt2 components",
    )
    use_servo_arg = DeclareLaunchArgument(
        "use_servo",
        default_value="false",
        description="If true, launch MoveIt Servo instead of move_group",
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
        default_value="NorthRx_",
        description="USB serial string for the left F9P (empty selects first match)",
    )
    right_gnss_serial_arg = DeclareLaunchArgument(
        "right_gnss_serial",
        default_value="TowerRx_",
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
    rtcm_input_topic_arg = DeclareLaunchArgument(
        "rtcm_input_topic",
        default_value="/base/rtcm",
        description="Topic providing RTCM corrections for rover F9Ps (set to /ntrip_client/rtcm to use NTRIP)",
    )
    enable_ntrip_arg = DeclareLaunchArgument(
        "enable_ntrip",
        default_value="false",
        description="Start NTRIP client for RTCM corrections",
    )
    ntrip_use_https_arg = DeclareLaunchArgument(
        "ntrip_use_https",
        default_value="false",
        description="Use HTTPS to reach NTRIP caster",
    )
    ntrip_host_arg = DeclareLaunchArgument(
        "ntrip_host",
        default_value="www.gnssdata.or.kr",
        description="NTRIP caster host",
    )
    ntrip_port_arg = DeclareLaunchArgument(
        "ntrip_port",
        default_value="2101",
        description="NTRIP caster port",
    )
    ntrip_mountpoint_arg = DeclareLaunchArgument(
        "ntrip_mountpoint",
        default_value="SEJN-RTCM32",
        description="NTRIP mountpoint",
    )
    ntrip_username_arg = DeclareLaunchArgument(
        "ntrip_username",
        default_value=EnvironmentVariable("NTRIP_USERNAME", default_value="gmmyung@kaist.ac.kr"),
        description="NTRIP username (can also set NTRIP_USERNAME env var)",
    )
    ntrip_password_arg = DeclareLaunchArgument(
        "ntrip_password",
        default_value=EnvironmentVariable("NTRIP_PASSWORD", default_value="gnss"),
        description="NTRIP password (can also set NTRIP_PASSWORD env var)",
    )
    ntrip_log_level_arg = DeclareLaunchArgument(
        "ntrip_log_level",
        default_value="INFO",
        description="Log level for NTRIP client",
    )
    ntrip_maxage_conn_arg = DeclareLaunchArgument(
        "ntrip_maxage_conn",
        default_value="30",
        description="Max age for reconnection attempts (seconds)",
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

    ntrip_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ublox_dgnss"), "launch", "ntrip_client.launch.py"]
            )
        ),
        launch_arguments={
            "use_https": LaunchConfiguration("ntrip_use_https"),
            "host": LaunchConfiguration("ntrip_host"),
            "port": LaunchConfiguration("ntrip_port"),
            "mountpoint": LaunchConfiguration("ntrip_mountpoint"),
            "username": LaunchConfiguration("ntrip_username"),
            "password": LaunchConfiguration("ntrip_password"),
            "log_level": LaunchConfiguration("ntrip_log_level"),
            "maxage_conn": LaunchConfiguration("ntrip_maxage_conn"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_ntrip")),
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
            "enable_manipulator": LaunchConfiguration("enable_manipulator"),
            "use_servo": LaunchConfiguration("use_servo"),
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
            "rtcm_input_topic": LaunchConfiguration("rtcm_input_topic"),
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
            "rtcm_input_topic": LaunchConfiguration("rtcm_input_topic"),
        }.items(),
    )

    # Stagger GNSS init to avoid simultaneous USB enumeration timeouts
    ublox_left_launch_delayed = TimerAction(
        period=40.0,
        actions=[ublox_left_launch],
    )

    ublox_right_launch_delayed = TimerAction(
        period=20.0,
        actions=[ublox_right_launch],
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

    # Static TF for rocker joints (hardware has no joint states for these)
    left_rocker_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="left_rocker_static_tf",
        arguments=["0", "0.2455", "0.06", "0", "0", "0", "base_chassis", "left_rocker"],
    )

    right_rocker_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="right_rocker_static_tf",
        arguments=["0", "-0.2455", "0.06", "0", "0", "0", "base_chassis", "right_rocker"],
    )

    # Manually set navsat datum if requested (after navsat_transform nodes start)
    manual_set_datum = TimerAction(
        period=10.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/datum",
                    "robot_localization/srv/SetDatum",
                    PythonExpression(
                        [
                            "'{geo_pose: {position: {latitude: ",
                            LaunchConfiguration("datum_lat"),
                            ", longitude: ",
                            LaunchConfiguration("datum_lon"),
                            ", altitude: ",
                            LaunchConfiguration("datum_alt"),
                            "}}}'",
                        ]
                    ),
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/datum",
                    "robot_localization/srv/SetDatum",
                    PythonExpression(
                        [
                            "'{geo_pose: {position: {latitude: ",
                            LaunchConfiguration("datum_lat"),
                            ", longitude: ",
                            LaunchConfiguration("datum_lon"),
                            ", altitude: ",
                            LaunchConfiguration("datum_alt"),
                            "}}}'",
                        ]
                    ),
                ],
                output="screen",
            ),
        ],
        condition=IfCondition(LaunchConfiguration("set_manual_datum")),
    )

    return LaunchDescription(
        [
            set_manual_datum_arg,
            datum_lat_arg,
            datum_lon_arg,
            datum_alt_arg,
            rviz_arg,
            controller_config_arg,
            can_iface_arg,
            use_mock_servos_arg,
            enable_manipulator_arg,
            use_servo_arg,
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
            rtcm_input_topic_arg,
            enable_ntrip_arg,
            ntrip_use_https_arg,
            ntrip_host_arg,
            ntrip_port_arg,
            ntrip_mountpoint_arg,
            ntrip_username_arg,
            ntrip_password_arg,
            ntrip_log_level_arg,
            ntrip_maxage_conn_arg,
            sik_sim_launch,
            realsense_launch,
            traversability_launch,
            ntrip_client_launch,
            rover_launch,
            ublox_left_launch_delayed,
            ublox_right_launch_delayed,
            left_navsat_relay,
            right_navsat_relay,
            left_rocker_static_tf,
            right_rocker_static_tf,
            manual_set_datum,
        ]
    )
