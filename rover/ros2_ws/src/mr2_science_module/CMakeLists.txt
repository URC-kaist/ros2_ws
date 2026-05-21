cmake_minimum_required(VERSION 3.10)
project(mr2_science_module LANGUAGES C CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(mr2_can_bus_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CarriageMotorTelemetry.msg"
  "srv/CarriageMotorCommand.srv"
  "srv/MoveCarriage.srv"
  "srv/MotorPosition.srv"
  "srv/MotorVelocity.srv"
  "srv/Pump.srv"
  "srv/SelectCentrifugePosition.srv"
  "srv/SetScienceLed.srv"
  "srv/TriggerCentrifugeRamp.srv"
  DEPENDENCIES builtin_interfaces)

ament_export_dependencies(rosidl_default_runtime)

add_executable(science_module_can_node src/science_module_can_node.cpp)
ament_target_dependencies(science_module_can_node
  mr2_can_bus_core
  rclcpp)

rosidl_get_typesupport_target(typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
if(typesupport_target)
  target_link_libraries(science_module_can_node ${typesupport_target})
endif()
add_dependencies(science_module_can_node ${PROJECT_NAME}__rosidl_generator_cpp)

install(TARGETS science_module_can_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch msg srv
  DESTINATION share/${PROJECT_NAME})

ament_package()
