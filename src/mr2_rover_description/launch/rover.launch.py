from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    desc_pkg   = FindPackageShare('mr2_rover_description')

    xacro_file   = PathJoinSubstitution([desc_pkg,   'urdf',   'rover.urdf.xacro'])
    ctrl_yaml    = PathJoinSubstitution([desc_pkg,   'config', 'controllers.yaml'])
    world_file   = PathJoinSubstitution([desc_pkg, 'worlds', 'world.sdf'])
    print(xacro_file)

    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # ───── Gazebo (use ros_gz_sim launcher) ──────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                  'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': [TextSubstitution(text='-r '), world_file]
        }.items()
    )

    # ───── robot_state_publisher & ros2_control_node ────────────────────
    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[robot_description])

    # ───── spawn the robot into Gazebo ───────────────────────────────────
    spawn = Node(package='ros_gz_sim',
                 executable='create',
                 arguments=['-topic', 'robot_description', '-name', 'rover', '-z', '5.0'])

    # ───── load controllers (after ros2_control is running) ─────────────
    jsb_spawner   = Node(package='controller_manager', executable='spawner',
                         arguments=['joint_state_broadcaster'])
    steer_spawner = Node(package='controller_manager', executable='spawner',
                         arguments=['steering_controller'])
    drive_spawner = Node(package='controller_manager', executable='spawner',
                         arguments=['drive_controller'])

    return LaunchDescription([
        gz_sim,
        rsp,
        TimerAction(period=2.0, actions=[spawn]),
        TimerAction(period=4.0, actions=[jsb_spawner, steer_spawner, drive_spawner]),
    ])

