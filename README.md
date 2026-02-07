### Quick memo on useful commands

```bash
ros2 run tf2_tools view_frames
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 doctor --report > doctor.log # detect QoS mode mismatch failure
# under ros2_ws:
rosdep install --from-paths src -y --ignore-src --rosdistro humble
# But this will not solve every dependency issue.
# ex: geographiclib, ros-humble-aruco-opencv-msgs, ...
ros2 pkg create --build-type ament_cmake <package_name>
```


Launch ros2_ws (please change sim to real for rover.):

```bash
# under rover/ros2_ws/
source install/setup.bash

# General entry
ros2 launch mr2_launch rover_sim.launch.py

ros2 launch mr2_launch rover_real.launch.py enable_manipulator:=false enable_sik_sim:=false

# Autonomous mission entry
# You MUST ensure that EVERYTHING is brought up by rover.launch.py beforehand.
ros2 launch mr2_rover_auto navigation.launch.py mode:=sim 2>&1 | tee navigation.log

ros2 launch mr2_rover_auto navigation.launch.py mode:=sim \
  --ros-args --log-level controller_server:=debug \
  2>&1 | tee navigation.log

```

Scripts to host and receive web:
Please install Node.js and npm!

``` bash
# Building dashboard:
# under scripts/ (requires sudo; installs nginx if missing)
# please set  
. deploy_dashboard.bash 2> dashboard_err.log

# Base station SIK interface:
# under base/gateway/
# run `npm install` to install dependencies
npm start -- --device /tmp/sik_sim1 --baud 57600 --port 8081
```

Nginx host mapping (for upstream like `mr2-ubuntu.local`):

```bash
# pick the correct IP for the upstream host (example: 192.168.1.50)
sudo sh -c 'printf "\n127.0.0.1 mr2-ubuntu.local\n" >> /etc/hosts'

# verify and reload nginx
sudo nginx -t
sudo systemctl reload nginx
```

### WGS84 Shift Helper (East/North meters -> lat/lon)

For quick/rough coordinate offsets on the WGS84 ellipsoid:
Note: rover initial coordinate in sim is (38.406738, -110.791397)... I think?
```bash
./scripts/wgs84_shift.py 38.406738 --110.791397 100 100

# or
./scripts/wgs84_shift.py --wgs84 "38.4065,-110.7919" 100 100 --format json
```

### Mission Master Topics (Publish + Monitor)

Mission Master subscribes to:
- `mission_list` (`mr2_action_interface/msg/MissionList`)
- `mission_control` (`mr2_action_interface/msg/MissionControl`)

Mission Master publishes:
- `mission_status` (`mr2_action_interface/msg/MissionStatus`)

Enums:
- `mission_type`: `0=UNKNOWN`, `1=GNSS_ONLY`, `2=COVER_VISION`
- `detection_method`: `0=NONE`, `1=ARUCO`, `2=YOLO`
- `object_type` (YOLO): `0=MALLET`, `1=PICK`, `2=BOTTLE`
- `command`: `0=NOOP`, `1=PAUSE`, `2=RESUME`, `3=ABORT`

Publish a GNSS-only mission list (single mission):
```bash
ros2 topic pub -1 /mission_list mr2_action_interface/msg/MissionList "{
  missions: [
    {mission_id: 1, mission_type: 1, detection_method: 0, object_type: 0,
     target_latitude: 38.4065, target_longitude: -110.7900,
     target_radius: 0.0, waypoint_count: 0}
  ]
}"
```

Publish a GNSS-only mission list (two missions):
```bash
ros2 topic pub -1 /mission_list mr2_action_interface/msg/MissionList "{
  missions: [
    {mission_id: 2, mission_type: 1, detection_method: 0, object_type: 0,
     target_latitude: 38.4067, target_longitude: -110.7900,
     target_radius: 0.0, waypoint_count: 0},
    {mission_id: 3, mission_type: 1, detection_method: 0, object_type: 0,
     target_latitude: 38.4065, target_longitude: -110.7900,
     target_radius: 0.0, waypoint_count: 0}
  ]
}"
```

Publish a CoverVision mission with ArUco:
```bash
ros2 topic pub -1 /mission_list mr2_action_interface/msg/MissionList "{
  missions: [
    {mission_id: 5, mission_type: 2, detection_method: 2, object_type: 0,
     target_latitude: 38.4065, target_longitude: -110.7900,
     target_radius: 5.0, waypoint_count: 0}
  ]
}"
```

Publish a CoverVision mission with ArUco and then YOLO:
```bash
ros2 topic pub -1 /mission_list mr2_action_interface/msg/MissionList "{
  missions: [
    {mission_id: 4, mission_type: 1, detection_method: 1, object_type: 0,
     target_latitude: 38.40645496, target_longitude: -110.7900,
     target_radius: 0.0, waypoint_count: 0},
    {mission_id: 5, mission_type: 2, detection_method: 2, object_type: 0,
     target_latitude: 38.4065, target_longitude: -110.7900,
     target_radius: 5.0, waypoint_count: 0}
  ]
}"
```

Pause (optionally clear costmaps), resume, abort: (Sent via Sik!)
```bash
ros2 topic pub -1 /mission_control mr2_action_interface/msg/MissionControl "{command: 1, clear_costmap: true, mission_id: 0}"
ros2 topic pub -1 /mission_control mr2_action_interface/msg/MissionControl "{command: 2, clear_costmap: false, mission_id: 0}"
ros2 topic pub -1 /mission_control mr2_action_interface/msg/MissionControl "{command: 3, clear_costmap: true, mission_id: 0}"
```

Monitor Mission Master:
```bash
# state: 0=IDLE, 1=RUNNING, 2=PAUSED, 3=COMPLETED, 4=FAILED
ros2 topic echo /mission_status
ros2 topic hz /mission_status
```

Notes:
- Missions execute in the order listed in `missions: [...]` (Mission Master does not loop the list).
- `mission_id` is currently used for status/debug correlation only (not ordering).
- `target_radius` / `waypoint_count` are only used by `COVER_VISION` missions; set them to `0` for `GNSS_ONLY`.
- `object_type` is used by `COVER_VISION` + `YOLO` (class id). For `GNSS_ONLY` or `ARUCO`, set to `0`.
- GNSS missions rely on `robot_localization/srv/FromLL` (WGS84 -> map) from `navsat_transform_node`. Debug:
```bash
ros2 service list | rg fromLL
ros2 service call /fromLL robot_localization/srv/FromLL "{ll_point: {latitude: 38.5, longitude: -110.8, altitude: 0.0}}"
```

### Vision Topics (ArUco + YOLO)

CoverVision mission consumes a single "mission-level" detection topic:
- `cover_vision/object_pose` (`geometry_msgs/PoseStamped`, **must be in `map` frame**)

Adapters (launched by `mr2_rover_auto/launch/action.launch.py`) bridge perception into that topic:
- YOLO adapter node: `cover_vision_yolo_adapter` subscribes `yolo/object_pose/class_<object_type>` -> publishes `cover_vision/object_pose`
- ArUco adapter node: `cover_vision_aruco_adapter` subscribes `aruco_detections` -> publishes `cover_vision/object_pose`

CoverVision coverage path (for RViz/debug):
- `cover_vision/coverage_path` (`nav_msgs/Path`, latched / transient-local)

Monitor mission-level detection:
```bash
ros2 topic echo /cover_vision/object_pose
ros2 topic echo /cover_vision/coverage_path
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
