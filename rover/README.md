# Rover

## Manual Dependencies

After running `rosdep` in `rover/ros2_ws`, one runtime dependency is still manual:

```bash
python3 -m pip install ultralytics
```

This is required by `mr2_yolo_perception` because
[yolo_rgbd_detector.py](./ros2_ws/src/mr2_yolo_perception/mr2_yolo_perception/yolo_rgbd_detector.py)
imports `from ultralytics import YOLO`, and there is no working `rosdep` key for it in this workspace.

## OpenCV/ROS Humble Repair Notes (Jetson)

### Problem

After removing Jetson OpenCV packages, builds can fail with:

```text
The imported target "opencv_core" references "/usr/lib/libopencv_core.so.4.8.0" but this file does not exist.
```

This happens when `libopencv-dev` still points to Jetson OpenCV 4.8 CMake files while ROS Humble packages (for example `cv_bridge`, `grid_map_*`) use OpenCV 4.5d.

### Recovery Steps

Run these commands in order:

```bash
sudo apt remove -y opencv-licenses
sudo dpkg --configure -a
sudo apt --fix-broken install -y
sudo apt install -y --allow-downgrades libopencv-dev=4.5.4+dfsg-9ubuntu4
```

### Verify

```bash
dpkg -l | egrep 'libopencv-dev|libopencv-core4.5d|libopencv-imgproc4.5d|libopencv-photo4.5d|opencv-licenses'
```

Expected:
- `libopencv-dev` is `4.5.4+dfsg-9ubuntu4`
- `libopencv-*-4.5d` runtime packages are installed
- `opencv-licenses` is not installed

### Rebuild Affected Package

```bash
cd ~/mr2-stack/rover/ros2_ws
rm -rf build/mr2_rover_auto install/mr2_rover_auto
source /opt/ros/humble/setup.bash
colcon build --packages-select mr2_rover_auto
```
