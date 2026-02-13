import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory("mr2_rover_auto")
    default_params = os.path.join(pkg_share, "config", "trav_pipeline.yaml")

    # Use a distinct launch argument name to avoid colliding with Nav2's
    # `params_file` (which broke map_server earlier).
    params_file_arg = DeclareLaunchArgument(
        "trav_params_file",
        default_value=default_params,
        description="Traversability pipeline parameters file",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation clock if true",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("trav_params_file")
    param_substitutions = {"use_sim_time": use_sim_time}
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    return LaunchDescription(
        [
            params_file_arg,
            use_sim_time_arg,
            Node(
                package="mr2_rover_auto",
                executable="pc2_to_heightmap_node",
                name="pc2_to_heightmap",
                output="screen",
                parameters=[configured_params],
            ),
            Node(
                package="mr2_rover_auto",
                executable="traversability_filter_node",
                name="grid_map_filters",
                output="screen",
                parameters=[configured_params],
            ),
            # Node(
            #     package="mr2_rover_auto",
            #     executable="gridmap_to_occupancy_node",
            #     name="gridmap_to_occupancy",
            #     output="screen",
            #     parameters=[configured_params],
            # ),
            # Node(
            #     package="mr2_rover_auto",
            #     executable="gridmap_to_pointcloud_node",
            #     name="gridmap_to_pointcloud",
            #     output="screen",
            #     parameters=[configured_params],
            # ),
        ]
    )
