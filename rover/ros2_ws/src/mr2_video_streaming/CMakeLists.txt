cmake_minimum_required(VERSION 3.8)
project(mr2_video_streaming)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

add_executable(video_streaming_node
  src/video_streaming_node.cpp
)
target_compile_features(video_streaming_node PUBLIC cxx_std_17)
target_link_libraries(video_streaming_node PkgConfig::GSTREAMER)
ament_target_dependencies(video_streaming_node
  rclcpp
  sensor_msgs
)

install(TARGETS video_streaming_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
