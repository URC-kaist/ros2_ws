cmake_minimum_required(VERSION 3.8)
project(mr2_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# https://stackoverflow.com/questions/48169653/finding-geographiclib-in-cmake-on-debian
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

add_executable(path_to_geopath_node src/path_to_geopath_node.cpp)
ament_target_dependencies(path_to_geopath_node
  rclcpp nav_msgs sensor_msgs geographic_msgs tf2 tf2_ros tf2_geometry_msgs
)
if(TARGET GeographicLib::GeographicLib)
  set(GEOGRAPHICLIB_LINK_TARGET GeographicLib::GeographicLib)
else()
  set(GEOGRAPHICLIB_LINK_TARGET ${GeographicLib_LIBRARIES})
  target_include_directories(path_to_geopath_node PRIVATE ${GeographicLib_INCLUDE_DIRS})
endif()
# Use plain signature to match ament_target_dependencies linkage.
target_link_libraries(path_to_geopath_node ${GEOGRAPHICLIB_LINK_TARGET})

install(TARGETS
  path_to_geopath_node
  DESTINATION lib/${PROJECT_NAME}/
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
