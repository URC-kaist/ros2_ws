cmake_minimum_required(VERSION 3.8)
project(mr2_latency_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UplinkTrialStatus.msg"
  "srv/AcquireVideoStreamLease.srv"
  "srv/ReleaseVideoStreamLease.srv"
  "srv/PrepareUplinkTrial.srv"
  "srv/StartUplinkTrial.srv"
  "srv/CancelUplinkTrial.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
