# .launch.py for dual GPS heading, estimator, WGS84-to-ENU query, and static tf for Nav2
# Make sure the sensors and their topics (imu/local_data, */fix) are all up!

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("mr2_rover_auto")
    params_file = os.path.join(pkg_share, "config", "estimator_params.yaml")

    return LaunchDescription([
        # For Gazebo, set to true. For field test, set to false.
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        #### A. Dual GPS Global heading calculation
        # 1) rewrite yaw of imu/local_data to imu/data
        Node(
            package='mr2_rover_auto',
            executable='gps_heading_imu_node',
            name='gps_heading_imu_node',
            output='screen',
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")}
            ],
            remappings=[('rover_north/fix', '/left_gnss/navsat'),
                        ('rover_south/fix', '/right_gnss/navsat'),
                        ('imu/local_data', '/imu')]
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
        Node( # Describe tf: odom -> base_link (Dummy identity tf for Nav2!)
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom_base_link',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',
                       # Equivalent
                       'odom', 'base_link']
        ), # XXX DO NOT USE odom frame for localization!

        # Please consult the graph:
        # https://docs.ros.org/en/noetic/api/robot_localization/html/integrating_gps.html
        # 3) GPS -> odometry/gps
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
            ],
        ),

        # 4) Single EKF: publish tf: map -> odom
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                params_file,
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
            remappings=[
                ("imu/data", "/imu/data"),
            ],
        ),

        #### B. Query node for goal pose coordinate conversion.
        # 5) Query for tf: GPS -> odometry/gps ((lat, long) -> ENU) with datum
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
        # 6) Define depth camera pose from base_link
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
