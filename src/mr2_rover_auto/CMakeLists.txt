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
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mr2_action_interface REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(gps_heading_imu_node src/gps_heading_imu_node.cpp)
ament_target_dependencies(gps_heading_imu_node
  rclcpp sensor_msgs
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

install(TARGETS
  gps_heading_imu_node
  gnss_only_server
  cover_vision_server
  gnss_only_client
  cover_vision_client
  DESTINATION lib/${PROJECT_NAME}/
)

install( DIRECTORY include DESTINATION include )
install( DIRECTORY launch DESTINATION share/${PROJECT_NAME}/ )
install( DIRECTORY config DESTINATION share/${PROJECT_NAME}/ )
install( DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME}/)
install( DIRECTORY maps DESTINATION share/${PROJECT_NAME}/ )

ament_package()
