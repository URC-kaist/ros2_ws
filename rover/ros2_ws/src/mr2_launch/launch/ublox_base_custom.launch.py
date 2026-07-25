"""
Launch a ublox_dgnss_node as a base with tunable survey-in parameters.

This lives outside the ublox_dgnss submodule to avoid modifying vendor code.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    log_level = LaunchConfiguration("log_level")
    device_family = LaunchConfiguration("device_family")
    namespace = LaunchConfiguration("namespace")
    device_serial_string = LaunchConfiguration("device_serial_string")
    frame_id = LaunchConfiguration("frame_id")
    tmode_mode = LaunchConfiguration("tmode_mode")
    svin_acc_limit = LaunchConfiguration("svin_acc_limit")
    svin_min_dur = LaunchConfiguration("svin_min_dur")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "log_level", default_value=TextSubstitution(text="INFO")
            ),
            DeclareLaunchArgument(
                "device_family", default_value=TextSubstitution(text="F9P")
            ),
            DeclareLaunchArgument("namespace", default_value="base"),
            DeclareLaunchArgument(
                "device_serial_string",
                default_value="",
                description="Serial string of the device to use",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="base",
                description="frame_id for published messages",
            ),
            DeclareLaunchArgument(
                "tmode_mode",
                default_value="0x1",
                description="CFG_TMODE_MODE (0=disabled, 1=survey-in, 2=fixed)",
            ),
            DeclareLaunchArgument(
                "svin_acc_limit",
                default_value="0xC350",
                description="CFG_TMODE_SVIN_ACC_LIMIT (mm); default 0xC350 = 50,000 mm (5 cm)",
            ),
            DeclareLaunchArgument(
                "svin_min_dur",
                default_value="0x3C",
                description="CFG_TMODE_SVIN_MIN_DUR (seconds); default 0x3C = 60 s",
            ),
            ComposableNodeContainer(
                name="ublox_dgnss_base_custom",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                arguments=["--ros-args", "--log-level", log_level],
                composable_node_descriptions=[
                    ComposableNode(
                        package="ublox_dgnss_node",
                        plugin="ublox_dgnss::UbloxDGNSSNode",
                        name="ublox_dgnss",
                        namespace=namespace,
                        parameters=[
                            {"DEVICE_FAMILY": device_family},
                            {"DEVICE_SERIAL_STRING": device_serial_string},
                            {"FRAME_ID": frame_id},
                            {"CFG_USBOUTPROT_NMEA": False},
                            # RTCM outputs
                            {"CFG_MSGOUT_RTCM_3X_TYPE1005_USB": 0x1},
                            {"CFG_MSGOUT_RTCM_3X_TYPE1077_USB": 0x1},
                            {"CFG_MSGOUT_RTCM_3X_TYPE1087_USB": 0x1},
                            {"CFG_MSGOUT_RTCM_3X_TYPE1127_USB": 0x1},
                            {"CFG_MSGOUT_RTCM_3X_TYPE1097_USB": 0x1},
                            {"CFG_MSGOUT_RTCM_3X_TYPE1230_USB": 0x1},
                            # Survey/fixed mode parameters
                            {"CFG_TMODE_MODE": tmode_mode},
                            {"CFG_TMODE_SVIN_ACC_LIMIT": svin_acc_limit},
                            {"CFG_TMODE_SVIN_MIN_DUR": svin_min_dur},
                            # Recommended UBX outputs
                            {"CFG_MSGOUT_UBX_NAV_SIG_USB": 0x1},
                            {"CFG_MSGOUT_UBX_NAV_PVT_USB": 0x1},
                            {"CFG_MSGOUT_UBX_NAV_POSLLH_USB": 0x1},
                            {"CFG_MSGOUT_UBX_NAV_RELPOSNED_USB": 0x1},
                            {"CFG_MSGOUT_UBX_NAV_STATUS_USB": 0x1},
                            {"CFG_MSGOUT_UBX_NAV_SVIN_USB": 0x1},
                        ],
                    )
                ],
            ),
        ]
    )
