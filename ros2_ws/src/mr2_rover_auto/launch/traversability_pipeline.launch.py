import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("mr2_rover_auto")
    filter_chain = os.path.join(pkg_share, "config", "traversability_filter_chain.yaml")

    cloud_topic_arg = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/rgbd_camera/points",
        description="Input point cloud topic",
    )
    base_frame_arg = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="Robot base frame",
    )
    map_frame_arg = DeclareLaunchArgument(
        "map_frame",
        default_value="base_link",
        description="Grid map frame",
    )
    x_forward_arg = DeclareLaunchArgument(
        "x_forward_m",
        default_value="3.0",
        description="Forward range in meters",
    )
    y_width_arg = DeclareLaunchArgument(
        "y_width_m",
        default_value="2.84",
        description="Lateral width in meters",
    )
    resolution_arg = DeclareLaunchArgument(
        "resolution",
        default_value="0.1",
        description="Grid resolution (m)",
    )
    voxel_arg = DeclareLaunchArgument(
        "voxel_size_m",
        default_value="0.05",
        description="Voxel size for downsampling (m)",
    )
    roi_z_arg = DeclareLaunchArgument(
        "roi_z_max_m",
        default_value="3.0",
        description="Max Z distance in camera frame (m)",
    )
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate_hz",
        default_value="5.0",
        description="Max grid map publish rate (Hz)",
    )
    layer_name_arg = DeclareLaunchArgument(
        "layer_name",
        default_value="elevation",
        description="Grid layer name for heightmap output",
    )
    input_topic_arg = DeclareLaunchArgument(
        "traversability_input",
        default_value="/traversability_gridmap",
        description="Grid map topic after filtering",
    )
    output_topic_arg = DeclareLaunchArgument(
        "traversability_output",
        default_value="/traversability_occupancy",
        description="Generated occupancy topic",
    )
    trav_layer_arg = DeclareLaunchArgument(
        "traversability_layer",
        default_value="traversability",
        description="Grid layer to convert to occupancy",
    )
    trav_min_arg = DeclareLaunchArgument(
        "traversability_min",
        default_value="0.0",
        description="Minimum traversability value mapped to free",
    )
    trav_max_arg = DeclareLaunchArgument(
        "traversability_max",
        default_value="1.0",
        description="Maximum traversability value mapped to free",
    )
    trav_invert_arg = DeclareLaunchArgument(
        "traversability_invert",
        default_value="true",
        description="Invert traversability when building occupancy",
    )
    trav_unknown_arg = DeclareLaunchArgument(
        "traversability_unknown",
        default_value="0",
        description="Unknown occupancy value",
    )

    return LaunchDescription(
        [
            cloud_topic_arg,
            base_frame_arg,
            map_frame_arg,
            x_forward_arg,
            y_width_arg,
            resolution_arg,
            voxel_arg,
            roi_z_arg,
            publish_rate_arg,
            layer_name_arg,
            input_topic_arg,
            output_topic_arg,
            trav_layer_arg,
            trav_min_arg,
            trav_max_arg,
            trav_invert_arg,
            trav_unknown_arg,
            Node(
                package="mr2_rover_auto",
                executable="pc2_to_heightmap_node",
                name="pc2_to_heightmap",
                output="screen",
                parameters=[
                    {
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
                    }
                ],
            ),
            Node(
                package="mr2_rover_auto",
                executable="traversability_filter_node",
                name="grid_map_filters",
                output="screen",
                parameters=[filter_chain],
            ),
            Node(
                package="mr2_rover_auto",
                executable="gridmap_to_occupancy_node",
                name="gridmap_to_occupancy",
                output="screen",
                parameters=[
                    {
                        "input_topic": LaunchConfiguration("traversability_input"),
                        "output_topic": LaunchConfiguration("traversability_output"),
                        "layer": LaunchConfiguration("traversability_layer"),
                        "min_value": LaunchConfiguration("traversability_min"),
                        "max_value": LaunchConfiguration("traversability_max"),
                        "invert": LaunchConfiguration("traversability_invert"),
                        "unknown_value": LaunchConfiguration("traversability_unknown"),
                    }
                ],
            ),
        ]
    )
