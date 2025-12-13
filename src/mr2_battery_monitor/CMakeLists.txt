cmake_minimum_required(VERSION 3.10)
project(mr2_battery_monitor LANGUAGES C CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mr2_can_bus_core REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PackTelemetry.msg"
  DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)

add_executable(battery_monitor_node src/battery_monitor_node.cpp)
ament_target_dependencies(battery_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
  mr2_can_bus_core)

rosidl_get_typesupport_target(typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
if(typesupport_target)
  target_link_libraries(battery_monitor_node ${typesupport_target})
endif()
add_dependencies(battery_monitor_node ${PROJECT_NAME}__rosidl_generator_cpp)

install(TARGETS battery_monitor_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY msg
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS scripts/emulate_battery.bash
  DESTINATION lib/${PROJECT_NAME})

ament_package()
