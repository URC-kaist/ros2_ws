from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    use_tensorrt = LaunchConfiguration("use_tensorrt").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    use_cuda = LaunchConfiguration("use_cuda").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    torch_device = LaunchConfiguration("torch_device").perform(context)
    publish_annotated = LaunchConfiguration("publish_annotated").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    annotated_fps = float(LaunchConfiguration("annotated_fps").perform(context))
    default_model_path = LaunchConfiguration("model_path").perform(context)
    tensorrt_engine_path = LaunchConfiguration("tensorrt_engine_path").perform(context)
    selected_model_path = tensorrt_engine_path if use_tensorrt else default_model_path
    selected_device = torch_device if (not use_tensorrt and use_cuda) else ""

    return [
        Node(
            package="mr2_yolo_perception",
            executable="yolo_rgbd_detector",
            name="yolo_rgbd_detector",
            output="screen",
            parameters=[
                {
                    "rgb_topic": "/rgbd_camera/color/image_raw",
                    "depth_topic": "/rgbd_camera/aligned_depth_to_color/image_raw",
                    "camera_info_topic": "/rgbd_camera/color/camera_info",
                    "annotated_topic": "yolo/annotated_image",
                    "publish_annotated": publish_annotated,
                    "annotated_fps": annotated_fps,
                    "pose_topic": "yolo/object_pose",
                    "target_frame": "",
                    "model_path": selected_model_path,
                    "conf_threshold": 0.25,
                    "iou_threshold": 0.45,
                    "device": selected_device,
                    "target_class": "",
                    "target_class_id": -1,
                    "class_ids": [0, 1, 2],
                    "class_id_map": "0:2,1:0,2:1",
                    "camera_frame_is_optical": False,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value="yolov11.pt",
                description="Default model weights path used when use_tensorrt is false.",
            ),
            DeclareLaunchArgument(
                "tensorrt_engine_path",
                default_value="~/urc_v2 small.engine",
                description="TensorRT engine weights path used when use_tensorrt is true.",
            ),
            DeclareLaunchArgument(
                "use_tensorrt",
                default_value="false",
                description="Set true to use tensorrt_engine_path for model_path.",
            ),
            DeclareLaunchArgument(
                "use_cuda",
                default_value="false",
                description="Set true to run torch mode on torch_device (ignored when use_tensorrt is true).",
            ),
            DeclareLaunchArgument(
                "torch_device",
                default_value="cuda:0",
                description="Torch device string (e.g. cuda:0, cuda:1, cpu) used when use_cuda is true.",
            ),
            DeclareLaunchArgument(
                "publish_annotated",
                default_value="true",
                description="Publish annotated YOLO debug images.",
            ),
            DeclareLaunchArgument(
                "annotated_fps",
                default_value="0.0",
                description="Maximum annotated YOLO debug image publish rate in Hz; 0 publishes every frame.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
