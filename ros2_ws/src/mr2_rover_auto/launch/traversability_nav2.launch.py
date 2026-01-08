import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("mr2_rover_auto")
    pipeline_launch = os.path.join(pkg_share, "launch", "traversability_pipeline.launch.py")
    nav2_launch = os.path.join(pkg_share, "launch", "nav2.launch.py")

    default_params = os.path.join(pkg_share, "config", "nav2_params.yaml")
    default_map = os.path.join(pkg_share, "maps", "map.yaml")

    launch_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use simulation clock if true",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Nav2 parameters file",
        ),
        DeclareLaunchArgument(
            "map",
            default_value=default_map,
            description="Map yaml for global costmap",
        ),
        DeclareLaunchArgument(
            "cloud_topic",
            default_value="/rgbd_camera/points",
            description="Input point cloud topic",
        ),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="Robot base frame",
        ),
        DeclareLaunchArgument(
            "map_frame",
            default_value="base_link",
            description="Grid map frame",
        ),
        DeclareLaunchArgument(
            "x_forward_m",
            default_value="3.0",
            description="Forward range in meters",
        ),
        DeclareLaunchArgument(
            "y_width_m",
            default_value="2.84",
            description="Lateral width in meters",
        ),
        DeclareLaunchArgument(
            "resolution",
            default_value="0.1",
            description="Grid resolution (m)",
        ),
        DeclareLaunchArgument(
            "voxel_size_m",
            default_value="0.05",
            description="Voxel size for downsampling (m)",
        ),
        DeclareLaunchArgument(
            "roi_z_max_m",
            default_value="3.0",
            description="Max Z distance in camera frame (m)",
        ),
        DeclareLaunchArgument(
            "publish_rate_hz",
            default_value="5.0",
            description="Max grid map publish rate (Hz)",
        ),
        DeclareLaunchArgument(
            "layer_name",
            default_value="elevation",
            description="Grid layer name for heightmap output",
        ),
        DeclareLaunchArgument(
            "traversability_input",
            default_value="/traversability_gridmap",
            description="Grid map topic after filtering",
        ),
        DeclareLaunchArgument(
            "traversability_output",
            default_value="/traversability_occupancy",
            description="Generated occupancy topic",
        ),
        DeclareLaunchArgument(
            "traversability_layer",
            default_value="traversability",
            description="Grid layer to convert to occupancy",
        ),
        DeclareLaunchArgument(
            "traversability_min",
            default_value="0.0",
            description="Minimum traversability value mapped to free",
        ),
        DeclareLaunchArgument(
            "traversability_max",
            default_value="1.0",
            description="Maximum traversability value mapped to free",
        ),
        DeclareLaunchArgument(
            "traversability_invert",
            default_value="true",
            description="Invert traversability when building occupancy",
        ),
        DeclareLaunchArgument(
            "traversability_unknown",
            default_value="0",
            description="Unknown occupancy value",
        ),
    ]

    pipeline_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(pipeline_launch),
        launch_arguments={
            "cloud_topic": LaunchConfiguration("cloud_topic"),
            "base_frame": LaunchConfiguration("base_frame"),
            "map_frame": LaunchConfiguration("map_frame"),
            "x_forward_m": LaunchConfiguration("x_forward_m"),
            "y_width_m": LaunchConfiguration("y_width_m"),
            "resolution": LaunchConfiguration("resolution"),
            "layer_name": LaunchConfiguration("layer_name"),
            "voxel_size_m": LaunchConfiguration("voxel_size_m"),
            "roi_z_max_m": LaunchConfiguration("roi_z_max_m"),
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            "traversability_input": LaunchConfiguration("traversability_input"),
            "traversability_output": LaunchConfiguration("traversability_output"),
            "traversability_layer": LaunchConfiguration("traversability_layer"),
            "traversability_min": LaunchConfiguration("traversability_min"),
            "traversability_max": LaunchConfiguration("traversability_max"),
            "traversability_invert": LaunchConfiguration("traversability_invert"),
            "traversability_unknown": LaunchConfiguration("traversability_unknown"),
        }.items(),
    )

    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "params_file": LaunchConfiguration("params_file"),
            "map": LaunchConfiguration("map"),
        }.items(),
    )

    return LaunchDescription(launch_args + [pipeline_include, nav2_include])
