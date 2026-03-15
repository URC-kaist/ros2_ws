from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    force_enable = LaunchConfiguration("force_enable")
    map_frame = LaunchConfiguration("map_frame")
    real_and_detector = LaunchConfiguration("real_and_detector")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "force_enable",
                default_value="false",
                description="Ignore mission status gating when true",
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value="map",
                description="Target frame for published detections",
            ),
            DeclareLaunchArgument(
                "real_and_detector",
                default_value="false",
                description="Launch detector nodes (YOLO + ArUco) for real hardware",
            ),
            DeclareLaunchArgument(
                "yolo_class_id",
                default_value="0",
                description="YOLO class id to subscribe to when force enabled",
            ),
            DeclareLaunchArgument(
                "yolo_pose_topic_prefix",
                default_value="yolo/object_pose",
                description="Prefix for YOLO pose topics",
            ),
            DeclareLaunchArgument(
                "yolo_cam_topic",
                default_value="/rgbd_camera",
                description="RealSense camera base topic for YOLO (e.g., /rgbd_camera)",
            ),
            DeclareLaunchArgument(
                "yolo_model_path",
                default_value="yolov11.pt",
                description="YOLO model path. Absolute paths are supported.",
            ),
            DeclareLaunchArgument(
                "yolo_output_topic",
                default_value="/vision_ablation/yolo/object_pose",
                description="Output topic for YOLO detections",
            ),
            DeclareLaunchArgument(
                "aruco_detections_topic",
                default_value="aruco_detections",
                description="Input topic for ArUco detections",
            ),
            DeclareLaunchArgument(
                "aruco_cam_topic",
                default_value="/front_camera/image_raw",
                description="Base image topic for ArUco detection (must have matching /camera_info)",
            ),
            DeclareLaunchArgument(
                "aruco_output_topic",
                default_value="/vision_ablation/aruco/object_pose",
                description="Output topic for ArUco detections",
            ),
            Node(
                package="aruco_opencv",
                executable="aruco_tracker_autostart",
                name="aruco_tracker",
                output="screen",
                condition=IfCondition(real_and_detector),
                parameters=[
                    PathJoinSubstitution(
                        [FindPackageShare("aruco_opencv"), "config", "aruco_tracker.yaml"]
                    ),
                    {
                        "cam_base_topic": LaunchConfiguration("aruco_cam_topic"),
                        "marker_size": 0.15,
                        "image_is_rectified": False,
                        "aruco.detectInvertedMarker": True,
                        "use_sim_time": use_sim_time,
                    },
                ],
            ),
            Node(
                package="mr2_yolo_perception",
                executable="yolo_rgbd_detector",
                name="yolo_detector",
                output="screen",
                condition=IfCondition(real_and_detector),
                parameters=[
                    {
                        "rgb_topic": PythonExpression(
                            ["'", LaunchConfiguration("yolo_cam_topic"), "/color/image_raw'"]
                        ),
                        "depth_topic": PythonExpression(
                            ["'", LaunchConfiguration("yolo_cam_topic"), "/aligned_depth_to_color/image_raw'"]
                        ),
                        "camera_info_topic": PythonExpression(
                            ["'", LaunchConfiguration("yolo_cam_topic"), "/color/camera_info'"]
                        ),
                        "annotated_topic": "yolo/annotated_image",
                        "pose_topic": LaunchConfiguration("yolo_pose_topic_prefix"),
                        "camera_frame_is_optical": real_and_detector,
                        "model_path": LaunchConfiguration("yolo_model_path"),
                        "class_id_map": "0:2,1:0,2:1",
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="mr2_rover_auto",
                executable="cover_vision_yolo_adapter",
                name="cover_vision_yolo_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "force_enable": force_enable,
                        "map_frame": map_frame,
                        "forced_class_id": LaunchConfiguration("yolo_class_id"),
                        "yolo_pose_topic_prefix": LaunchConfiguration("yolo_pose_topic_prefix"),
                        "output_topic": LaunchConfiguration("yolo_output_topic"),
                    }
                ],
            ),
            Node(
                package="mr2_rover_auto",
                executable="cover_vision_aruco_adapter",
                name="cover_vision_aruco_adapter",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "force_enable": force_enable,
                        "map_frame": map_frame,
                        "aruco_detections_topic": LaunchConfiguration("aruco_detections_topic"),
                        "output_topic": LaunchConfiguration("aruco_output_topic"),
                    }
                ],
            ),
        ]
    )
