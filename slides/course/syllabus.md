# MR2 ROS 2 코드 구조 워크스루 실라버스

대상은 MR2 rover codebase를 유지보수하거나 subsystem 개발에 참여할 사람들이다. 이 repo는 web frontend, networking, backend gateway, ROS 2, navigation, hardware firmware, 운영 스크립트가 한곳에 있는 monorepo다. 목표는 모든 파일을 암기하는 것이 아니라, 기능 요청이나 장애가 들어왔을 때 어느 층의 코드를 봐야 하는지 빠르게 판단할 수 있게 만드는 것이다.

권장 포맷은 회차당 60-90분이다. 매회 `목적 -> 패키지 지도 -> launch entrypoint -> 주요 인터페이스 -> 핵심 코드 -> 실행/디버깅` 순서로 진행한다.

## 1회차: 전체 시스템 구조와 테크 스택

목표:

- MR2 시스템이 dashboard, nginx, base gateway, radio/video network, rover ROS 2 stack, CAN/device firmware로 어떻게 나뉘는지 이해한다.
- 사용자의 브라우저 조작이 어떤 경로로 rover command, ROS service/action, video stream, telemetry로 이어지는지 설명할 수 있게 한다.
- 이후 회차에서 코드를 읽을 때 "이 변경은 web/base/rover/hardware 중 어디를 건드리는가"를 판단할 수 있게 한다.

주요 내용:

- 기본 용어: client/server/process/port/path/proxy, HTTP/HTTPS/TLS, WebSocket, Node.js, React/Vite
- Web UI: `dashboard` React/Vite app, roslib, MapLibre, Three.js, uPlot
- Reverse proxy/deploy: `scripts/deploy_dashboard.bash`, nginx static serving, TLS, `/xbee-ws`, `/video-ws`, `/rosbridge-ws`, `/tiles`
- Base gateway: `base/gateway` Node.js process, WebSocket hub, serial/XBee bridge, video gateway, Rocket M2 status, antenna tracker, optional ROS topic relay
- Rover bridge: `mr2_xbee_bridge`, `rosbridge_server`, rover-side ROS graph
- Rover ROS 2: launch, description, ros2_control, Nav2, localization, perception, payload nodes
- Hardware I/O: CAN devices, u-blox GNSS/RTK, cameras, science module, base station Arduino/antenna, Rocket M2 network gear
- Build/runtime split: npm/Vite/Node for web and gateway, colcon/ament for ROS, nginx/systemd/Linux deployment

데모:

- dashboard URL path가 nginx에서 어디로 proxy되는지 확인한다.
- dashboard client file이 `/xbee-ws`, `/video-ws`, `/rosbridge-ws` 중 무엇을 쓰는지 찾는다.
- gateway가 XBee serial, video stream, ROS relay, Rocket M2 status를 어떻게 조립하는지 본다.

과제:

- 각자 기능 하나를 골라 브라우저 UI부터 rover hardware 또는 ROS node까지의 end-to-end 경로를 5분 안에 설명한다.

## 2회차: 로봇 모델, 시뮬레이션, 런치 시스템

목표:

- 로버가 URDF/Xacro, TF, controller, Gazebo world로 어떻게 조립되는지 이해한다.
- real/sim launch 차이를 파악한다.

주요 내용:

- `mr2_rover_description`
  - `urdf/rover.urdf.xacro`
  - `urdf/rover_base.xacro`
  - `urdf/swerve_module.xacro`
  - `urdf/manipulator.xacro`
  - `urdf/rover_gazebo_sensors.xacro`
- controller config
  - `config/controllers/rover_controllers.yaml`
  - `config/controllers/manipulator_controllers.yaml`
- launch 구성
  - `mr2_rover_description/launch/real.launch.py`
  - `mr2_rover_description/launch/sim.launch.py`
  - `mr2_launch/launch/rover_real.launch.py`
  - `mr2_launch/launch/rover_sim.launch.py`
- Gazebo bridge
  - `config/gz_bridge_topics.yaml`
  - `/clock`, IMU, camera, GNSS bridge
- macOS Gazebo GUI 제한과 server-only 운영상 주의점

데모:

- `robot_state_publisher`와 TF tree 확인
- `ros2 control list_controllers`
- `ros2 control list_hardware_interfaces`

과제:

- link/joint 하나를 골라 Xacro에서 TF와 controller까지 연결 경로를 추적한다.

## 3회차: 주행 제어와 CAN 하드웨어 계층

목표:

- 속도 명령이 controller와 hardware interface를 거쳐 CAN 장치로 내려가는 흐름을 이해한다.
- 실제 hardware-only 코드와 mock/emulator 코드를 구분한다.

주요 내용:

- `mr2_rover_control`
  - `twist_to_commands_controller.cpp`
  - controller plugin 구조
  - command/state interface
- `mr2_can_bus_core`
  - `can_bus_manager`
  - `can_bus_registry`
  - `can_device`
- `mr2_can_hardware_interface`
  - `can_hw.cpp`
  - `ros2_control` hardware interface
- device packages
  - `mr2_devices_ak_servo`
  - `mr2_devices_output_actuator`
  - `mr2_camera_turret`
  - `mr2_led`
  - `mr2_battery_monitor`

데모:

- `/cmd_vel` 또는 `/base/cmd_vel` publish
- controller state/command topic echo
- battery emulator topic 확인

과제:

- "명령 하나가 들어와서 CAN frame까지 가는 경로"를 시퀀스 다이어그램으로 정리한다.

## 4회차: 자율주행, 위치추정, 지도, Nav2

목표:

- mission/action layer와 Nav2/localization/perception pipeline의 연결 구조를 이해한다.

주요 내용:

- `mr2_rover_auto`
  - `mission_master.cpp`
  - `gnss_only_server.cpp`
  - `cover_vision_server.cpp`
  - `gps_heading_gps_node.cpp`
  - `base_datum_setter.cpp`
- interface contracts
  - `GnssOnly.action`
  - `CoverVision.action`
  - `PanoramaCapture.action`
  - `MissionStatus.msg`
  - `MissionControl.msg`
- localization
  - `dual_ekf_navsat.yaml`
  - `dual_ekf_navsat_real.yaml`
  - `ekf_imu_real.yaml`
- Nav2
  - `nav2.launch.py`
  - `navigation.launch.py`
  - `nav2_params.yaml`
  - behavior trees
- traversability
  - `traversability_pipeline.launch.py`
  - `trav_pipeline.yaml`
  - `mr2_nav2_plugins/traversability_layer.cpp`
- GNSS/RTK
  - `ublox_dgnss`
  - `rtcm_msgs`
  - NTRIP/RTCM correction flow

데모:

- `ros2 action list`
- localization launch 구성 확인
- map/traversability topic 흐름 확인

과제:

- `GnssOnly` 또는 `CoverVision` action 하나의 goal/result/feedback과 관련 노드를 설명한다.

## 5회차: Payload, 비전, 운영/디버깅, 개발 규칙

목표:

- science, spectrophotometer, vision, video, manipulator, status 패키지의 위치를 이해한다.
- 실제 개발/운영 중 장애를 추적하는 공통 루틴을 정리한다.

주요 내용:

- science module
  - `mr2_science_module`
  - motor, pump, centrifuge, LED service
- spectrophotometer
  - `mr2_spectrophotometer`
  - calibration/config
  - `GetSpectrum.srv`
- perception/video
  - `mr2_yolo_perception`
  - `mr2_video_streaming`
  - `mr2_panorama`
- MoveIt/manipulator
  - `mr2_moveit`
  - SRDF, joint limits, servo config
- system status and orchestration
  - `mr2_system_status`
  - `mr2_launch`
- debugging routines
  - build failure: dependency, `package.xml`, `CMakeLists.txt`
  - runtime failure: node list, topic echo, logs
  - controller failure: `ros2 control`
  - TF failure: frame tree
  - hardware failure: CAN node/device layer

데모:

- 기능 하나를 실제 launch하고 topic/service/action 확인
- 장애 상황 하나를 잡고 node/topic/service/log/config 순서로 추적

마무리 산출물:

- 패키지별 owner 또는 담당자
- 새 기능 추가 체크리스트
- subsystem별 "먼저 봐야 하는 파일" 목록
