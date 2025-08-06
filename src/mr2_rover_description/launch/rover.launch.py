from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc_pkg = FindPackageShare("mr2_rover_description")

    xacro_file = PathJoinSubstitution([desc_pkg, "urdf", "rover.urdf.xacro"])
    ctrl_yaml = PathJoinSubstitution([desc_pkg, "config", "controllers.yaml"])
    bridge_yaml = PathJoinSubstitution([desc_pkg, "config", "gz_bridge.yaml"])
    world_file = PathJoinSubstitution([desc_pkg, "worlds", "world.sdf"])

    robot_description = {"robot_description": Command(["xacro ", xacro_file])}

    headless = LaunchConfiguration("headless")
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run Gazebo without GUI",
    )

    # ───── Gazebo (use ros_gz_sim launcher) ──────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [TextSubstitution(text="-r "), world_file]
        }.items(),
        condition=UnlessCondition(headless),
    )
    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={
            "gz_args": [
                TextSubstitution(text="-r --headless-rendering -s "),
                world_file,
            ]
        }.items(),
        condition=IfCondition(headless),
    )

    # ───── robot_state_publisher & ros2_control_node ────────────────────
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
    )

    # ───── spawn the robot into Gazebo ───────────────────────────────────
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "rover", "-z", "0.0"],
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/left_gnss/navsat@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat",
            "/right_gnss/navsat@sensor_msgs/msg/NavSatFix[ignition.msgs.NavSat",
            "/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "/rgbd_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/rgbd_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/rgbd_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
        ],
        output="screen",
    )

    # ───── load controllers (after ros2_control is running) ─────────────
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    rover_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rover_controller"],
    )

    return LaunchDescription(
        [
            headless_arg,
            gz_sim,
            gz_sim_headless,
            rsp,
            TimerAction(period=2.0, actions=[spawn]),
            TimerAction(
                period=6.0, actions=[jsb_spawner, rover_controller_spawner]
            ),
            gz_bridge,
        ]
    )
