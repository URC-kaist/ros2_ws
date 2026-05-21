import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _yaml_to_dict(path_to_yaml: str) -> dict:
    with open(path_to_yaml, "r") as f:
        return yaml.safe_load(f) or {}


def _as_bool(value) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _set_if_present(params: dict, context, launch_name: str, param_name: str) -> None:
    value = LaunchConfiguration(launch_name).perform(context)
    if value != "":
        params[param_name] = value


def _apply_imu_only_params(params: dict) -> None:
    params.update(
        {
            "enable_color": False,
            "enable_depth": False,
            "enable_infra": False,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_rgbd": False,
            "enable_gyro": True,
            "enable_accel": True,
            "enable_motion": False,
            "pointcloud.enable": False,
            "pointcloud__neon_.enable": False,
            "align_depth.enable": False,
            "colorizer.enable": False,
        }
    )


def _launch_setup(context):
    config_path = PathJoinSubstitution(
        [FindPackageShare("mr2_launch"), "config", "realsense_rgbd.yaml"]
    ).perform(context)
    params = _yaml_to_dict(config_path)

    _set_if_present(params, context, "camera_name", "camera_name")
    _set_if_present(params, context, "camera_namespace", "camera_namespace")
    _set_if_present(params, context, "base_frame_id", "base_frame_id")
    _set_if_present(params, context, "log_level", "log_level")

    if _as_bool(LaunchConfiguration("imu_only").perform(context)):
        _apply_imu_only_params(params)

    if "rgb_camera.power_line_frequency" not in params:
        params["rgb_camera.power_line_frequency"] = 2

    camera_name = str(params.get("camera_name", "rgbd_camera"))
    base_frame_id = str(params.get("base_frame_id", "rgbd_camera"))
    urdf_mount_frame = LaunchConfiguration("urdf_mount_frame").perform(context)
    if urdf_mount_frame == "":
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
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_name", default_value=""),
            DeclareLaunchArgument("camera_namespace", default_value=""),
            DeclareLaunchArgument("base_frame_id", default_value=""),
            DeclareLaunchArgument("urdf_mount_frame", default_value=""),
            DeclareLaunchArgument("log_level", default_value="info"),
            DeclareLaunchArgument(
                "imu_only",
                default_value="false",
                description=(
                    "Disable RealSense image/depth streams and keep gyro/accel "
                    "IMU streams only"
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
