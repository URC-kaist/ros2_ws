"""Wrapper launch for ublox fixed-base + rover setup with unique container names."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")
    device_family = LaunchConfiguration("device_family")
    namespace = LaunchConfiguration("namespace")
    device_serial_string = LaunchConfiguration("device_serial_string")
    frame_id = LaunchConfiguration("frame_id")
    rtcm_input_topic = LaunchConfiguration("rtcm_input_topic")

    log_level_arg = DeclareLaunchArgument(
        "log_level", default_value=TextSubstitution(text="INFO")
    )
    device_family_arg = DeclareLaunchArgument(
        "device_family", default_value=TextSubstitution(text="F9P")
    )
    namespace_arg = DeclareLaunchArgument("namespace", default_value="rover")
    device_serial_string_arg = DeclareLaunchArgument(
        "device_serial_string",
        default_value="",
        description="Serial string of the device to use",
    )
    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="rover",
        description="The frame_id to use in header of published messages",
    )
    rtcm_input_topic_arg = DeclareLaunchArgument(
        "rtcm_input_topic",
        default_value="/base/rtcm",
        description="Topic supplying RTCM corrections (e.g., /base/rtcm or /ntrip_client/rtcm)",
    )

    params = [
        {"DEVICE_FAMILY": device_family},
        {"DEVICE_SERIAL_STRING": device_serial_string},
        {"FRAME_ID": frame_id},
        {"CFG_USBOUTPROT_NMEA": False},
        {"CFG_RATE_MEAS": 10},
        {"CFG_RATE_NAV": 100},
        {"CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB": 1},
        {"CFG_MSGOUT_UBX_NAV_COV_USB": 1},
        {"CFG_MSGOUT_UBX_RXM_RTCM_USB": 1},
        {"CFG_MSGOUT_UBX_NAV_SIG_USB": 1},
        {"CFG_MSGOUT_UBX_NAV_PVT_USB": 1},
        {"CFG_MSGOUT_UBX_NAV_POSLLH_USB": 1},
        {"CFG_MSGOUT_UBX_NAV_RELPOSNED_USB": 1},
        {"CFG_MSGOUT_UBX_NAV_STATUS_USB": 1},
        {"CFG_MSGOUT_UBX_NAV_SVIN_USB": 1},
        {"CFG_MSGOUT_UBX_NAV_SOL_USB": 1},
    ]

    # Run as standalone executables instead of components to avoid
    # component_container_mt shutdown/runtime segfaults under rapid restarts.
    dgnss_node = Node(
        package="ublox_dgnss_node",
        executable="ublox_dgnss_node",
        name="ublox_dgnss",
        namespace=namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=params,
        remappings=[
            ("/ntrip_client/rtcm", rtcm_input_topic),
        ],
    )

    nav_sat_fix_hp_node = Node(
        package="ublox_nav_sat_fix_hp_node",
        executable="ublox_nav_sat_fix_hp",
        name="ublox_nav_sat_fix_hp",
        namespace=namespace,
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
    )

    return LaunchDescription(
        [
            log_level_arg,
            device_family_arg,
            namespace_arg,
            device_serial_string_arg,
            frame_id_arg,
            rtcm_input_topic_arg,
            dgnss_node,
            nav_sat_fix_hp_node,
        ]
    )
