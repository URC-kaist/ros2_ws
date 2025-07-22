#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare(package='ros_gz_sim')

    # file:// URI for empty.sdf
    world_uri = [
        TextSubstitution(text='file://'),
        PathJoinSubstitution([pkg_share, 'worlds', 'empty.sdf'])
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={
            'gz_args': [
                '-r',                  # run immediately
                '-v4',                 # verbosity
                '--render-engine',     # specify render engine
                'ogre',
                *world_uri            # world URI
            ]
        }.items()
    )

    return LaunchDescription([gz_sim])
