cmake_minimum_required(VERSION 3.8)
project(mr2_drive_motor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_executable(mr2_drive_motor_interface src/mr2_drive_motor_interface.cpp)
ament_target_dependencies(mr2_drive_motor_interface
  rclcpp
  sensor_msgs
  std_srvs
)
target_link_libraries(mr2_drive_motor_interface
  Boost::system
)

install(TARGETS
  mr2_drive_motor_interface
  DESTINATION lib/${PROJECT_NAME}
)

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

ament_package()
