cmake_minimum_required(VERSION 3.8)
project(mr2_rover_auto)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mr2_action_interface REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(filters REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${PCL_INCLUDE_DIRS})
# Grid map eigen plugin headers live in include/grid_map_core/eigen_plugins
# Ensure all targets see the grid_map_core include directory.
include_directories(${grid_map_core_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(gps_heading_imu_node src/gps_heading_imu_node.cpp)
ament_target_dependencies(gps_heading_imu_node
  rclcpp sensor_msgs
  tf2 tf2_geometry_msgs Eigen3
)

add_executable(gps_heading_gps_node src/gps_heading_gps_node.cpp)
ament_target_dependencies(gps_heading_gps_node
  rclcpp sensor_msgs nav_msgs
  tf2 tf2_geometry_msgs Eigen3
)

add_executable(gnss_only_server src/gnss_only_server.cpp)
ament_target_dependencies(gnss_only_server
rclcpp rclcpp_action nav2_msgs geometry_msgs nav_msgs sensor_msgs
tf2_ros tf2_geometry_msgs mr2_action_interface
)

add_executable(cover_vision_server src/cover_vision_server.cpp)
ament_target_dependencies(cover_vision_server
rclcpp rclcpp_action nav2_msgs geometry_msgs nav_msgs sensor_msgs
tf2_ros tf2_geometry_msgs mr2_action_interface
)

add_executable(gnss_only_client src/gnss_only_client.cpp)
ament_target_dependencies(gnss_only_client
rclcpp rclcpp_action mr2_action_interface
)

add_executable(cover_vision_client src/cover_vision_client.cpp)
ament_target_dependencies(cover_vision_client
rclcpp rclcpp_action mr2_action_interface
)

add_executable(wheel_encoder_odom_node src/wheel_encoder_odom_node.cpp)
ament_target_dependencies(wheel_encoder_odom_node
  rclcpp sensor_msgs nav_msgs Eigen3
)

add_executable(pc2_to_heightmap_node src/pc2_to_heightmap_node.cpp)
ament_target_dependencies(pc2_to_heightmap_node
  rclcpp sensor_msgs grid_map_msgs grid_map_core tf2 tf2_ros tf2_eigen pcl_conversions Eigen3
)
target_link_libraries(pc2_to_heightmap_node ${PCL_LIBRARIES})

add_executable(traversability_filter_node src/traversability_filter_node.cpp)
ament_target_dependencies(traversability_filter_node
  rclcpp grid_map_ros grid_map_core grid_map_filters filters pluginlib
)

add_executable(gridmap_to_occupancy_node src/gridmap_to_occupancy_node.cpp)
ament_target_dependencies(gridmap_to_occupancy_node
  rclcpp grid_map_msgs grid_map_core nav_msgs tf2 tf2_ros tf2_geometry_msgs
)

install(TARGETS
  gps_heading_imu_node
  gps_heading_gps_node
  gnss_only_server
  cover_vision_server
  gnss_only_client
  cover_vision_client
  wheel_encoder_odom_node
  pc2_to_heightmap_node
  traversability_filter_node
  gridmap_to_occupancy_node
  DESTINATION lib/${PROJECT_NAME}/
)

install( DIRECTORY include DESTINATION include )
install( DIRECTORY launch DESTINATION share/${PROJECT_NAME}/ )
install( DIRECTORY config DESTINATION share/${PROJECT_NAME}/ )
install( DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME}/)
install( DIRECTORY maps DESTINATION share/${PROJECT_NAME}/ )

ament_package()
