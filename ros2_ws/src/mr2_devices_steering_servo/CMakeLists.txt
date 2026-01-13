cmake_minimum_required(VERSION 3.10)
project(mr2_devices_steering_servo LANGUAGES CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(mr2_can_bus_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(steering_servo_device SHARED
  src/steering_servo_device.cpp
  src/can_protocol.cpp)
ament_target_dependencies(steering_servo_device
  mr2_can_bus_core hardware_interface pluginlib rclcpp)

target_include_directories(steering_servo_device PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(steering_fw_node
  src/main.cpp
  src/can_transport.cpp
  src/can_protocol.cpp
  src/can_service.cpp)

ament_target_dependencies(steering_fw_node mr2_can_bus_core rclcpp)

target_include_directories(steering_fw_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# export plugin
pluginlib_export_plugin_description_file(mr2_can_bus_core plugin.xml)

install(TARGETS steering_servo_device
        LIBRARY DESTINATION lib)

install(TARGETS steering_fw_node
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(FILES plugin.xml
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/)

ament_package()
