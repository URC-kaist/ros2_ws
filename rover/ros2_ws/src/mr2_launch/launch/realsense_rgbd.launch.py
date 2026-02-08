from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
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
        default_value="rgbd_camera",
        description="Base frame ID for the RealSense TF tree",
    )
    urdf_mount_frame_arg = DeclareLaunchArgument(
        "urdf_mount_frame",
        default_value="rgbd_camera",
        description="URDF frame the RealSense should be attached to",
    )
    color_profile_arg = DeclareLaunchArgument(
        "color_profile",
        default_value="640x480x6",
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
    unite_imu_method_arg = DeclareLaunchArgument(
        "unite_imu_method",
        default_value="2",
        description="Method to unite IMU data",
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
                "unite_imu_method": LaunchConfiguration("unite_imu_method"),
                "log_level": LaunchConfiguration("log_level"),
            }
        ],
        output="screen",
    )
    # RealSense frame naming uses camera_name-prefixed frame IDs.
    # Bridge URDF mount frame -> RealSense base frame so depth/pointcloud TF is connected.
    realsense_base_frame = PythonExpression(
        [
            "'",
            LaunchConfiguration("camera_name"),
            "_",
            LaunchConfiguration("base_frame_id"),
            "'",
        ]
    )
    realsense_mount_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="rgbd_camera_mount_tf",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            LaunchConfiguration("urdf_mount_frame"),
            "--child-frame-id",
            realsense_base_frame,
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
            urdf_mount_frame_arg,
            color_profile_arg,
            enable_depth_arg,
            align_depth_arg,
            pointcloud_arg,
            enable_gyro_arg,
            enable_accel_arg,
            unite_imu_method_arg,
            log_level_arg,
            realsense_node,
            realsense_mount_tf,
        ]
    )
