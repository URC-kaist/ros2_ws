from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mr2_launch_share = FindPackageShare("mr2_launch")
    rover_description_share = FindPackageShare("mr2_rover_description")
    rover_auto_share = FindPackageShare("mr2_rover_auto")

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution([mr2_launch_share, "rviz", "sim.rviz"]),
        ],
        output="screen",
    )

    include_real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([rover_description_share, "launch", "real.launch.py"])]
        )
    )

    include_realsense_rgbd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([mr2_launch_share, "launch", "realsense_rgbd.launch.py"])]
        )
    )

    include_local_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([rover_auto_share, "launch", "local_imu.launch.py"])]
        )
    )

    include_traversability_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([rover_auto_share, "launch", "traversability_pipeline.launch.py"])]
        )
    )

    include_front_uvc_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([mr2_launch_share, "launch", "front_uvc_camera.launch.py"])]
        )
    )

    include_vision_ablation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([rover_auto_share, "launch", "vision_ablation.launch.py"])]
        ),
        launch_arguments={
            "force_enable": "true",
            "real_and_detector": "true",
            "yolo_class_id": "0",
        }.items(),
    )

    return LaunchDescription([
        rviz2,
        include_real_launch,
        include_realsense_rgbd_launch,
        include_local_imu_launch,
        include_traversability_pipeline_launch,
        include_front_uvc_camera_launch,
        include_vision_ablation_launch
    ])
