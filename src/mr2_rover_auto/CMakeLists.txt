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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mr2_action_interface REQUIRED)

# Executable
add_executable(gps_heading_imu_node src/gps_heading_imu_node.cpp)
ament_target_dependencies(gps_heading_imu_node
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  Eigen3
)

# Action server and client
add_executable(gnssonly_action_server src/gnssonly_action_server.cpp)
ament_target_dependencies(gnssonly_action_server
  ament_index_cpp
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav2_msgs
  mr2_action_interface
  tf2
  tf2_geometry_msgs
  nav2_bringup
)
add_executable(covervision_action_server src/covervision_action_server.cpp)
ament_target_dependencies(covervision_action_server
  ament_index_cpp
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  nav2_msgs
  mr2_action_interface
  tf2
  tf2_geometry_msgs
  nav2_bringup
)
add_executable(mission_action_client src/mission_action_client.cpp)
ament_target_dependencies(mission_action_client
  rclcpp
  rclcpp_action
  rclcpp_components
  mr2_action_interface
  std_msgs
  nav2_bringup
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  gps_heading_imu_node
  DESTINATION lib/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

