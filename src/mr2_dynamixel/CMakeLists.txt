cmake_minimum_required(VERSION 3.8)
project(mr2_dynamixel)

# Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(dynamixel_controller src/dynamixel_controller.cpp)
ament_target_dependencies(dynamixel_controller
  rclcpp
  sensor_msgs
  dynamixel_sdk
)

install(TARGETS
  dynamixel_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
