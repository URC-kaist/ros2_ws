from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_name_arg = DeclareLaunchArgument(
        "camera_name",
        default_value="front_camera",
        description="Camera name for node and image topics",
    )

    camera_namespace_arg = DeclareLaunchArgument(
        "camera_namespace",
        default_value="front_camera",
        description="Namespace for camera topics",
    )

    video_device_arg = DeclareLaunchArgument(
        "video_device",
        default_value="/dev/videoFRONT",
        description="Video4Linux device for the front camera",
    )

    camera_info_url_arg = DeclareLaunchArgument(
        "camera_info_url",
        default_value="file:///home/mr2/mr2-stack/rover/ros2_ws/src/mr2_launch/config/ost.yaml",
        description="URL for the camera calibration file (ost.yaml)",
    )

    frame_id_arg = DeclareLaunchArgument(
        "frame_id",
        default_value="front_camera_optical_frame",
        description="Frame ID stamped on published images (should exist in URDF TF tree)",
    )
    # UVC camera driver
    front_camera_node = Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        name=LaunchConfiguration("camera_name"),
        namespace=LaunchConfiguration("camera_namespace"),
        output="screen",
        parameters=[
            {
                "video_device": LaunchConfiguration("video_device"),
                "camera_info_url": LaunchConfiguration("camera_info_url"),
                "camera_frame_id": LaunchConfiguration("frame_id"),
                "image_size": [848, 480],
                "output_encoding": "bgr8",
                "time_per_frame": [1, 10],
            }
        ],
    )

    return LaunchDescription(
        [
            camera_name_arg,
            camera_namespace_arg,
            video_device_arg,
            camera_info_url_arg,
            frame_id_arg,
            front_camera_node,
        ]
    )
