cmake_minimum_required(VERSION 3.8)
project(mr2_drive)

# Export compile commands
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(driver_node src/driver_node.cpp)
add_executable(steering_node src/steering_node.cpp)
ament_target_dependencies(driver_node rclcpp std_msgs sensor_msgs std_srvs geometry_msgs)
ament_target_dependencies(steering_node rclcpp std_msgs sensor_msgs Eigen3 geometry_msgs)

ament_export_dependencies(eigen3_cmake_module)
ament_export_dependencies(Eigen3)

install(TARGETS
  driver_node
  steering_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
