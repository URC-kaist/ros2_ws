cmake_minimum_required(VERSION 3.10)
project(mr2_led LANGUAGES C CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mr2_can_bus_core REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetLedMode.srv")

ament_export_dependencies(rosidl_default_runtime)

add_executable(led_can_node src/led_can_node.cpp)
ament_target_dependencies(led_can_node
  rclcpp
  mr2_can_bus_core)

rosidl_get_typesupport_target(typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
if(typesupport_target)
  target_link_libraries(led_can_node ${typesupport_target})
endif()
add_dependencies(led_can_node ${PROJECT_NAME}__rosidl_generator_cpp)

install(TARGETS led_can_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY srv
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
