cmake_minimum_required(VERSION 3.5)
project(mr2_rover_auto)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Executable
add_executable(gps_heading_imu_node src/gps_heading_imu_node.cpp)

ament_target_dependencies(gps_heading_imu_node
  rclcpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  Eigen3
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

ament_package()

