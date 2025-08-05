cmake_minimum_required(VERSION 3.10)
project(mr2_devices_ak_servo LANGUAGES CXX)

find_package(ament_cmake REQUIRED)
find_package(mr2_can_bus_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(ak_servo_device SHARED src/ak_servo_device.cpp)
ament_target_dependencies(ak_servo_device
  mr2_can_bus_core hardware_interface pluginlib rclcpp)

# export plugin
pluginlib_export_plugin_description_file(mr2_can_bus_core plugin.xml)

install(TARGETS ak_servo_device
        LIBRARY DESTINATION lib)

ament_package()
