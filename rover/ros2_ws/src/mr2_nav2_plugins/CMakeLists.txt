cmake_minimum_required(VERSION 3.8)
project(mr2_nav2_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)

add_library(mr2_nav2_plugins_core SHARED
  src/traversability_layer.cpp
)

ament_target_dependencies(mr2_nav2_plugins_core
  rclcpp
  nav2_costmap_2d
  pluginlib
  grid_map_ros
  grid_map_core
  grid_map_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  nav2_util
)

target_include_directories(mr2_nav2_plugins_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

pluginlib_export_plugin_description_file(nav2_costmap_2d traversability_layer.xml)

install(
  DIRECTORY include/
  DESTINATION include)

install(
  TARGETS mr2_nav2_plugins_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  FILES traversability_layer.xml
  DESTINATION share/${PROJECT_NAME})

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
