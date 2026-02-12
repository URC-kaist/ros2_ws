from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    force_enable = LaunchConfiguration("force_enable")
    map_frame = LaunchConfiguration("map_frame")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "force_enable",
                default_value="true",
                description="Ignore mission status gating when true",
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value="map",
                description="Target frame for published detections",
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
                "aruco_output_topic",
                default_value="/vision_ablation/aruco/object_pose",
                description="Output topic for ArUco detections",
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
