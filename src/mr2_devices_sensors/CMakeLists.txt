cmake_minimum_required(VERSION 3.10)
project(mr2_devices_sensors LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(mr2_can_bus_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${PROJECT_NAME} SHARED
  src/absolute_encoder_device.cpp
  src/limit_switch_device.cpp)

ament_target_dependencies(${PROJECT_NAME}
  mr2_can_bus_core
  hardware_interface
  pluginlib
  rclcpp)

pluginlib_export_plugin_description_file(mr2_can_bus_core plugin.xml)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()
