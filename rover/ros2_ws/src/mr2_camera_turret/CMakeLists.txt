cmake_minimum_required(VERSION 3.10)
project(mr2_camera_turret LANGUAGES C CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mr2_can_bus_core REQUIRED)

add_executable(camera_turret_can_node src/camera_turret_can_node.cpp)
ament_target_dependencies(camera_turret_can_node
  rclcpp
  geometry_msgs
  mr2_can_bus_core)

install(TARGETS camera_turret_can_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
