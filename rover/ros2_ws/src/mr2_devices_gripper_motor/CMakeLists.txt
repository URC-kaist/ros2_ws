cmake_minimum_required(VERSION 3.10)
project(mr2_devices_gripper_motor LANGUAGES CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(mr2_can_bus_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(gripper_motor_device SHARED
  src/gripper_motor_device.cpp
  src/can_protocol.cpp)
ament_target_dependencies(gripper_motor_device
  mr2_can_bus_core hardware_interface pluginlib rclcpp)

target_include_directories(gripper_motor_device PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

pluginlib_export_plugin_description_file(mr2_can_bus_core plugin.xml)

install(TARGETS gripper_motor_device
        LIBRARY DESTINATION lib)

install(FILES plugin.xml
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/)

ament_package()
