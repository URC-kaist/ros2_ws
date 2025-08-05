cmake_minimum_required(VERSION 3.10)
project(mr2_can_hardware_interface LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(mr2_can_bus_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/can_hw.cpp)

ament_target_dependencies(${PROJECT_NAME}
  mr2_can_bus_core
  hardware_interface
  pluginlib
  rclcpp)

pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib)


ament_package()
