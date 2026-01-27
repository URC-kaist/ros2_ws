from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="mr2_yolo_perception",
                executable="yolo_rgbd_detector",
                name="yolo_rgbd_detector",
                output="screen",
                parameters=[
                    {
                        "rgb_topic": "/rgbd_camera/color/image_raw",
                        "depth_topic": "/rgbd_camera/depth/image_rect_raw",
                        "camera_info_topic": "/rgbd_camera/color/camera_info",
                        "annotated_topic": "yolo/annotated_image",
                        "pose_topic": "yolo/object_pose",
                        "target_frame": "",
                        "model_path": "yolov11.pt",
                        "conf_threshold": 0.25,
                        "iou_threshold": 0.45,
                        "device": "",
                        "target_class": "",
                        "target_class_id": -1,
                        "class_ids": [0, 1, 2],
                        "camera_frame_is_optical": False,
                    }
                ],
            )
        ]
    )
