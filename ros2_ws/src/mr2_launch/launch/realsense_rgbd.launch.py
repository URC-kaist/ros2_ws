from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="rgbd_camera",
        description="RealSense camera name (used in topic names)",
    )
    camera_namespace_arg = DeclareLaunchArgument(
        "camera_namespace",
        default_value="",
        description="Namespace for RealSense topics (empty for root)",
    )
    serial_no_arg = DeclareLaunchArgument(
        "serial_no",
        default_value="''",
        description="Select device by serial number (empty for any)",
    )
    usb_port_id_arg = DeclareLaunchArgument(
        "usb_port_id",
        default_value="''",
        description="Select device by USB port id (empty for any)",
    )
    device_type_arg = DeclareLaunchArgument(
        "device_type",
        default_value="d435i",
        description="Select device by type (e.g., d435, d455)",
    )
    base_frame_id_arg = DeclareLaunchArgument(
        "base_frame_id",
        default_value="front_camera",
        description="Attach camera TF tree to this frame",
    )
    camera_frame_id_arg = DeclareLaunchArgument(
        "camera_frame_id",
        default_value="rgbd_camera_front_camera",
        description="Frame ID for the camera link (child of base_frame_id)",
    )
    color_profile_arg = DeclareLaunchArgument(
        "color_profile",
        default_value="640x480x30",
        description="Color stream profile (width x height x fps)",
    )
    enable_depth_arg = DeclareLaunchArgument(
        "enable_depth",
        default_value="true",
        description="Enable depth stream",
    )
    align_depth_arg = DeclareLaunchArgument(
        "align_depth_enable",
        default_value="true",
        description="Enable depth alignment to color",
    )
    pointcloud_arg = DeclareLaunchArgument(
        "pointcloud_enable",
        default_value="true",
        description="Enable pointcloud output",
    )
    enable_gyro_arg = DeclareLaunchArgument(
        "enable_gyro",
        default_value="true",
        description="Enable IMU gyro stream",
    )
    enable_accel_arg = DeclareLaunchArgument(
        "enable_accel",
        default_value="true",
        description="Enable IMU accel stream",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Log level for realsense2_camera node",
    )

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        name=LaunchConfiguration("camera_name"),
        namespace=LaunchConfiguration("camera_namespace"),
        parameters=[
            {
                "camera_name": LaunchConfiguration("camera_name"),
                "serial_no": LaunchConfiguration("serial_no"),
                "usb_port_id": LaunchConfiguration("usb_port_id"),
                "device_type": LaunchConfiguration("device_type"),
                "base_frame_id": LaunchConfiguration("base_frame_id"),
                "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
                "enable_depth": LaunchConfiguration("enable_depth"),
                "align_depth.enable": LaunchConfiguration("align_depth_enable"),
                "pointcloud__neon_.enable": LaunchConfiguration("pointcloud_enable"),
                "enable_gyro": LaunchConfiguration("enable_gyro"),
                "enable_accel": LaunchConfiguration("enable_accel"),
                "rgb_camera.power_line_frequency": 2,
                "log_level": LaunchConfiguration("log_level"),
            }
        ],
        output="screen",
    )
    camera_base_frame = LaunchConfiguration("base_frame_id")
    camera_link_frame = LaunchConfiguration("camera_frame_id")
    camera_tf_link = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="rgbd_camera_front_camera_tf",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            camera_base_frame,
            camera_link_frame,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            camera_name_arg,
            camera_namespace_arg,
            serial_no_arg,
            usb_port_id_arg,
            device_type_arg,
            base_frame_id_arg,
            camera_frame_id_arg,
            color_profile_arg,
            enable_depth_arg,
            align_depth_arg,
            pointcloud_arg,
            enable_gyro_arg,
            enable_accel_arg,
            log_level_arg,
            realsense_node,
            camera_tf_link,
        ]
    )
