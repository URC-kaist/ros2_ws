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
        }.items(),
    )

    # Static TF for rocker joints (hardware has no joint states for these)
    left_rocker_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="left_rocker_static_tf",
        arguments=[
            "--x",
            "0",
            "--y",
            "0.2455",
            "--z",
            "0.06",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "base_chassis",
            "--child-frame-id",
            "left_rocker",
        ],
    )

    right_rocker_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="right_rocker_static_tf",
        arguments=[
            "--x",
            "0",
            "--y",
            "-0.2455",
            "--z",
            "0.06",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "base_chassis",
            "--child-frame-id",
            "right_rocker",
        ],
    )

    return LaunchDescription(
        [
            rviz2,
            real_launch,
            realsense_rgbd_launch,
            local_imu_launch,
            traversability_pipeline_launch,
            front_uvc_camera_launch,
            vision_ablation_launch,
            left_rocker_static_tf,
            right_rocker_static_tf,
        ]
    )
