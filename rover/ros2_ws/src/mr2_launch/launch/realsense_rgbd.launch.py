import platform
import yaml
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _yaml_to_dict(path_to_yaml: str) -> dict:
    with open(path_to_yaml, "r") as f:
        return yaml.safe_load(f) or {}


def _launch_setup(context):
    config_path = PathJoinSubstitution(
        [FindPackageShare("mr2_launch"), "config", "realsense_rgbd.yaml"]
    ).perform(context)
    params = _yaml_to_dict(config_path)

    if "rgb_camera.power_line_frequency" not in params:
        params["rgb_camera.power_line_frequency"] = 2

    # Jetson uses pointcloud__neon_.enable; desktop uses pointcloud.enable.
    # If both are present, prefer NEON on aarch64.
    has_neon = "pointcloud__neon_.enable" in params
    has_std = "pointcloud.enable" in params
    if has_neon and has_std and platform.machine() == "aarch64":
        params.pop("pointcloud.enable", None)
    elif has_neon and has_std:
        params.pop("pointcloud__neon_.enable", None)

    camera_name = str(params.get("camera_name", "rgbd_camera"))
    base_frame_id = str(params.get("base_frame_id", "rgbd_camera"))
    urdf_mount_frame = str(params.get("urdf_mount_frame", "rgbd_camera"))

    return [
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name=camera_name,
            namespace=str(params.get("camera_namespace", "")),
            parameters=[params],
            output="screen",
            arguments=[
                "--ros-args",
                "--log-level",
                str(params.get("log_level", "info")),
            ],
        ),
        Node(
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
                urdf_mount_frame,
                "--child-frame-id",
                PythonExpression([
                    "'",
                    camera_name,
                    "_",
                    base_frame_id,
                    "'",
                ]),
            ],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_launch_setup)])
