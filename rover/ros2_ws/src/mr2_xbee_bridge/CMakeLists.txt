cmake_minimum_required(VERSION 3.8)
project(mr2_xbee_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mr2_action_interface REQUIRED)
find_package(mr2_battery_monitor REQUIRED)
find_package(ublox_ubx_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
add_library(${PROJECT_NAME} src/packets.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

add_executable(xbee_bridge_node src/xbee_bridge_node.cpp)
ament_target_dependencies(
  xbee_bridge_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  mr2_action_interface
  mr2_battery_monitor
  ublox_ubx_msgs
  rtcm_msgs
)
target_link_libraries(xbee_bridge_node ${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
target_compile_features(xbee_bridge_node PUBLIC cxx_std_17)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME} xbee_bridge_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


ament_package()
