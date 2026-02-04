cmake_minimum_required(VERSION 3.10)
project(mr2_spectrophotometer_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Spectrum.msg"
  "srv/GetSpectrum.srv"
  DEPENDENCIES std_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
