from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mr2_launch_share = FindPackageShare("mr2_launch")
    rover_description_share = FindPackageShare("mr2_rover_description")
    rover_auto_share = FindPackageShare("mr2_rover_auto")
    yolo_model_path = LaunchConfiguration("yolo_model_path")

    yolo_model_path_arg = DeclareLaunchArgument(
        "yolo_model_path",
        default_value="yolov11.pt",
        description="YOLO model path. Absolute paths are supported.",
    )

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

    real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([rover_description_share, "launch", "real.launch.py"])]
        ),
        launch_arguments={
            "enable_manipulator_module": "false",
        }.items(),
    )

    realsense_rgbd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([mr2_launch_share, "launch", "realsense_rgbd.launch.py"])]
        )
    )

    local_imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([rover_auto_share, "launch", "local_imu.launch.py"])]
        )
    )

    traversability_pipeline_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [rover_auto_share, "launch", "traversability_pipeline.launch.py"]
                )
            ]
        )
    )

    front_uvc_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([mr2_launch_share, "launch", "front_uvc_camera.launch.py"])]
        )
    )

    vision_ablation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([rover_auto_share, "launch", "vision_ablation.launch.py"])]
        ),
        launch_arguments={
            "force_enable": "true",
            "real_and_detector": "true",
            "yolo_class_id": "0",
            "yolo_model_path": yolo_model_path,
        }.items(),
    )

    passive_rocker_joint_state = Node(
        package="mr2_rover_description",
        executable="static_joint_state_publisher",
        name="passive_rocker_joint_state",
        output="screen",
        parameters=[
            {"joint_names": ["left_rocker_joint"]},
            {"positions": [0.0]},
            {"publish_rate": 10.0},
        ],
    )

    return LaunchDescription(
        [
            yolo_model_path_arg,
            rviz2,
            real_launch,
            realsense_rgbd_launch,
            local_imu_launch,
            traversability_pipeline_launch,
            front_uvc_camera_launch,
            vision_ablation_launch,
            passive_rocker_joint_state,
        ]
    )
