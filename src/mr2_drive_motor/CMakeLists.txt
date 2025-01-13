cmake_minimum_required(VERSION 3.8)
project(mr2_drive_motor)

# Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pid.msg"
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(mr2_drive_motor_interface src/mr2_drive_motor_interface.cpp)

ament_target_dependencies(mr2_drive_motor_interface
  rclcpp
  sensor_msgs
  std_srvs
  rosidl_default_runtime  
)

rosidl_get_typesupport_target(cpp_typesupport_target 
  ${PROJECT_NAME} 
  "rosidl_typesupport_cpp"
)

target_link_libraries(mr2_drive_motor_interface
  Boost::system
  "${cpp_typesupport_target}"
)

install(TARGETS
  mr2_drive_motor_interface
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
