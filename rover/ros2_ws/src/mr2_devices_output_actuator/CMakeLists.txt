cmake_minimum_required(VERSION 3.10)
project(mr2_devices_output_actuator LANGUAGES CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(mr2_can_bus_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(output_actuator_device SHARED
  src/output_actuator_device.cpp)
ament_target_dependencies(output_actuator_device
  mr2_can_bus_core hardware_interface pluginlib rclcpp)

pluginlib_export_plugin_description_file(mr2_can_bus_core plugin.xml)

install(TARGETS output_actuator_device
        LIBRARY DESTINATION lib)

install(FILES plugin.xml
        DESTINATION share/${PROJECT_NAME})

ament_package()
