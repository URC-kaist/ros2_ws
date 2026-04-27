cmake_minimum_required(VERSION 3.10)
project(mr2_devices_output_actuator LANGUAGES C CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(mr2_can_bus_core REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OutputAngleStatus.msg"
  "msg/OutputVelocityStatus.msg"
  "msg/TravelLimitsStatus.msg"
  "msg/ActuatorConfigStatus.msg"
  "msg/RuntimeDiagnostic.msg"
  DEPENDENCIES builtin_interfaces)

ament_export_dependencies(rosidl_default_runtime)

add_library(output_actuator_device SHARED
  src/output_actuator_device.cpp)
ament_target_dependencies(output_actuator_device
  mr2_can_bus_core hardware_interface pluginlib rclcpp)

rosidl_get_typesupport_target(typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
if(typesupport_target)
  target_link_libraries(output_actuator_device ${typesupport_target})
endif()
add_dependencies(output_actuator_device ${PROJECT_NAME}__rosidl_generator_cpp)

pluginlib_export_plugin_description_file(mr2_can_bus_core plugin.xml)

install(TARGETS output_actuator_device
        LIBRARY DESTINATION lib)

install(FILES plugin.xml
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY msg
        DESTINATION share/${PROJECT_NAME})

ament_package()
