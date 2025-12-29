# .launch.py for dual GPS heading, dual EKF localization, WGS84-to-ENU query, and static tf for Nav2
# Make sure the sensors and their topics (imu/data, */fix) are all up!

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("mr2_rover_auto")
    params_file = os.path.join(pkg_share, "config", "dual_ekf_navsat.yaml")

    return LaunchDescription([
        # For Gazebo, set to true. For field test, set to false.
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        #### A. Dual GPS Global heading calculation
        # 1) compute dual-GNSS yaw and publish into GPS odometry stream
        Node(
            package='mr2_rover_auto',
            executable='gps_heading_gps_node',
            name='gps_heading_gps_node',
            output='screen',
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
            remappings=[('rover_north/fix', '/left_gnss/navsat'),
                        ('rover_south/fix', '/right_gnss/navsat'),
                        ('imu/data', '/imu/data'),
                        ('odometry/gps/raw', '/odometry/gps/raw'),
                        ('odometry/gps', '/odometry/gps')]
        ),

        #### B. Estimator for robot_localization
        # 2) static_tf
        Node( # Describe IMU mount
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_imu',
            arguments=['0.0', '0.19', '0.0', '0.0', '0.0', '0.0',
                       # behind below of robot center
                       'base_link', 'imu_link']
        ),
        Node( # Describe Northern GPS mount
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_gps_north',
            arguments=['0.0', '0.245', '0.0', '0.0', '0.0', '0.0',
                       # left of robot center
                       'base_link', 'gps_north_link']
        ),

        # Wheel encoder odometry from ros2_control joint_states
        Node(
            package='mr2_rover_auto',
            executable='wheel_encoder_odom_node',
            name='wheel_encoder_odom_node',
            output='screen',
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                # Force absolute topics so remapping/namespace does not break inputs
                {"joint_state_topic": "/joint_states"},
                {"wheel_odom_topic": "/wheel_encoder/odometry"},
            ],
        ),

        # Please consult the graph:
        # https://docs.ros.org/en/noetic/api/robot_localization/html/integrating_gps.html
        # 3) GPS -> odometry/gps/raw (navsat_transform output)
        TimerAction(period=2.0, actions=[
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
            remappings=[
                # ("gps/fix", "rover_north/fix"),
                ("gps/fix", "/right_gnss/navsat"),
                ("imu/data", "/imu/data"),
                ("odometry/gps", "/odometry/gps/raw"),
            ],
        ),

        # 4) Local EKF: publish tf: odom -> base_link
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_local",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ("imu/data", "/imu/data"),
            ],
        ),

        # 5) Global EKF: publish tf: map -> odom
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_global",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ("imu/data", "/imu/data"),
                ("odometry/gps", "/odometry/gps"),
            ],
        ),

        #### B. Query node for goal pose coordinate conversion.
        # 6) Query for tf: GPS -> odometry/gps ((lat, long) -> ENU) with datum
        # XXX MUST share same datum!!!
        Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_query",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
            remappings=[
                ("gps/fix", "query/fix"),
                ("odometry/gps", "query/gps"),
                ("imu/data", "/imu/data"),
            ],
        )
        ]),
        
        #### C. Traversability tf (pitch -30 deg, for now)
        # 7) Define depth camera pose from base_link
        Node( # Describe Eastern Camera mount
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_cam_east',
            arguments=['0.1', '0.0', '0.0', '0.0', '0.3491', '0.0',
                       # front of robot, facing front, tilted toward ground
                       'base_link', 'cam_east_link']
        ),
        Node( # Describe Northen Camera mount
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_cam_north',
            arguments=['0.0', '0.1', '0.0', '0.0', '0.3491', '1.570796',
                       # left of robot, facing left, tilted toward ground
                       'base_link', 'cam_north_link']
        ),
        Node( # Describe Southern Camera mount
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_cam_south',
            arguments=['0.0', '-0.1', '0.0', '0.0', '0.3491', '-1.570796',
                       # left of robot, facing left, tilted toward ground
                       'base_link', 'cam_south_link']
        ),
    ])
