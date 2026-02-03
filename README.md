### Quick memo on useful commands

```bash
ros2 run tf2_tools view_frames
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 doctor --report > doctor.log # detect QoS mode mismatch failure
```


Launch ros2_ws (please change sim to real for rover.):

```bash
# under rover/ros2-ws/
source install/setup.bash

# General entry
ros2 launch mr2_launch rover.launch.py mode:=sim

# Autonomous mission entry
# You MUST ensure that EVERYTHING is brought up by rover.launch.py beforehand.
ros2 launch mr2_rover_auto navigation.launch.py mode:=sim 2>&1 | tee navigation.log

ros2 launch mr2_rover_auto navigation.launch.py mode:=sim \
  --ros-args --log-level controller_server:=debug \
  2>&1 | tee navigation.log

```

Scripts to host and recieve web:

``` bash
# Rover hosting:
# under rover/
. scripts/deploy_dashboard.bash 2> dashboard_err

# Base station SIK interface:
# under base/gateway/
npm start -- --device /tmp/sik_sim1 --baud 57600 --port 8081
```

### Mission Master Topics (Publish + Monitor)

Mission Master subscribes to:
- `mission_list` (`mr2_action_interface/msg/MissionList`)
- `mission_control` (`mr2_action_interface/msg/MissionControl`)

Mission Master publishes:
- `mission_status` (`mr2_action_interface/msg/MissionStatus`)

Enums:
- `mission_type`: `0=UNKNOWN`, `1=GNSS_ONLY`, `2=COVER_VISION`
- `detection_method`: `0=NONE`, `1=YOLO`, `2=ARUCO`
- `command`: `0=NOOP`, `1=PAUSE`, `2=RESUME`, `3=ABORT`

Publish a GNSS-only mission list (single mission):
```bash
ros2 topic pub -1 /mission_list mr2_action_interface/msg/MissionList "{
  missions: [
    {mission_id: 1, mission_type: 1, detection_method: 0,
     target_latitude: 38.4065, target_longitude: -110.7919,
     target_radius: 0.0, waypoint_count: 0}
  ]
}"
```

Publish a mixed mission list (GNSS-only, then CoverVision with YOLO):
```bash
ros2 topic pub -1 /mission_list mr2_action_interface/msg/MissionList "{
  missions: [
    {mission_id: 10, mission_type: 1, detection_method: 0,
     target_latitude: 38.4065, target_longitude: -110.7919,
     target_radius: 0.0, waypoint_count: 0},
    {mission_id: 11, mission_type: 2, detection_method: 1,
     target_latitude: 38.4066, target_longitude: -110.7921,
     target_radius: 5.0, waypoint_count: 12}
  ]
}"
```

Pause (optionally clear costmaps), resume, abort:
```bash
ros2 topic pub -1 /mission_control mr2_action_interface/msg/MissionControl "{command: 1, clear_costmap: true, mission_id: 0}"
ros2 topic pub -1 /mission_control mr2_action_interface/msg/MissionControl "{command: 2, clear_costmap: false, mission_id: 0}"
ros2 topic pub -1 /mission_control mr2_action_interface/msg/MissionControl "{command: 3, clear_costmap: true, mission_id: 0}"
```

Monitor Mission Master:
```bash
ros2 topic echo /mission_status
ros2 topic hz /mission_status
```

### Vision Topics (YOLO + ArUco)

YOLO RGBD detector (`mr2_yolo_perception/yolo_rgbd_detector.py`) topic defaults:
- Inputs (params): `rgb_topic=/rgbd_camera/color/image_raw`, `depth_topic=/rgbd_camera/depth/image_rect_raw`, `camera_info_topic=/rgbd_camera/color/camera_info`
- Outputs (params): `annotated_topic=yolo/annotated_image`, `pose_topic=yolo/object_pose`
- Published poses are per class: `yolo/object_pose/class_<id>` (`geometry_msgs/PoseStamped`)
- TF frames published per class: `yolo/class_<id>`

Useful checks:
```bash
ros2 param get /yolo_rgbd_detector rgb_topic
ros2 topic echo /yolo/object_pose/class_0
ros2 topic echo /yolo/annotated_image
```

ArUco tracker (`aruco_opencv/aruco_tracker_autostart`) notes:
- Input image base topic is set by `aruco_cam_topic` in `rover.launch.py` (default `/rgbd_camera/color/image_raw`)
- The base topic must have matching `/camera_info` (see comment in `rover.launch.py`)
- Output topics are defined by `aruco_opencv`; list them at runtime:
```bash
ros2 node info /aruco_tracker
ros2 topic list | rg aruco
```
- If `publish_tf` is enabled in `aruco_opencv` config, TF frames are published for detected markers/boards:
```bash
ros2 run tf2_tools view_frames
```
