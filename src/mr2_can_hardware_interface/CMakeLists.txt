cmake_minimum_required(VERSION 3.10)
project(mr2_can_hardware_interface LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(mr2_can_bus_core REQUIRED)
find_package(mr2_devices_sensors REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(transmission_interface REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/can_hw.cpp
  src/homing/absolute_encoder_policy.cpp
  src/homing/limit_switch_policy.cpp)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  mr2_can_bus_core
  mr2_devices_sensors
  hardware_interface
  pluginlib
  rclcpp
  transmission_interface)
target_link_libraries(${PROJECT_NAME}
  ${mr2_devices_sensors_LIBRARIES})

pluginlib_export_plugin_description_file(hardware_interface plugin.xml)
pluginlib_export_plugin_description_file(mr2_can_hardware_interface plugin.xml)

install(
  TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)


ament_package()
