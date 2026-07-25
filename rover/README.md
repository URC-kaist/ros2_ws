# Rover

## Direct manual operation

`mr2_launch/rover_direct.launch.py` is the real-hardware entry point for
Jetson-hosted operation without a base-station computer. It starts
the local XBEE PTY pair, Node.js gateway, drive/steering, battery/system status,
the rover XBEE bridge, rosbridge, and video streaming to `127.0.0.1`. The
manipulator and its active controllers are enabled by default.

It bypasses the GNSS/NTRIP/autonomy/science composition in
`rover_real.launch.py`. Direct `ros2 launch` is the primary operating mode:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mr2_launch rover_direct.launch.py
```

For drive-only operation when the manipulator hardware is not installed or
powered:

```bash
ros2 launch mr2_launch rover_direct.launch.py \
  enable_manipulator_module:=false
```

Optional direct-mode features can be enabled independently:

```bash
ros2 launch mr2_launch rover_direct.launch.py \
  enable_manipulator_module:=false \
  enable_video_streaming:=false \
  enable_led:=true \
  enable_camera_turret:=true
```

Use `ros2 launch mr2_launch rover_direct.launch.py --show-args` for the complete
parameter list.

For the complete Jetson AP, gateway/video relay, nginx, and systemd procedure,
see [`docs/rover-direct-operation.md`](../docs/rover-direct-operation.md).

## Manual Dependencies

After running `rosdep` in `rover/ros2_ws`, one runtime dependency is still manual:

```bash
python3 -m pip install ultralytics
```

This is required by `mr2_yolo_perception` because
[yolo_rgbd_detector.py](./ros2_ws/src/mr2_yolo_perception/mr2_yolo_perception/yolo_rgbd_detector.py)
imports `from ultralytics import YOLO`, and there is no working `rosdep` key for it in this workspace.

## Video Streaming Prerequisites

The rover-side video path depends on GStreamer development and runtime packages.
At minimum, the rover machine needs:

```bash
sudo apt install -y \
  libgstreamer1.0-dev \
  libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly \
  gstreamer1.0-plugins-bad
```

On Jetson, hardware H.264 encoding also needs NVIDIA's GStreamer package:

```bash
sudo apt install -y nvidia-l4t-gstreamer
```

Verify the required elements:

```bash
gst-inspect-1.0 h264parse
gst-inspect-1.0 nvv4l2h264enc
gst-inspect-1.0 nvvidconv
```

The central stream mapping lives at
[`ros2_ws/src/mr2_launch/config/video_streams.json`](./ros2_ws/src/mr2_launch/config/video_streams.json).
That file is the source of truth for:

- ROS image topic to stream ID mapping
- UDP port allocation
- expected ROS encoding (`rgb8` / `bgr8`)
- optional display metadata consumed by the dashboard

## Launching Rover Video Streaming

You can launch the video streamer directly:

```bash
cd ~/mr2-stack/rover/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mr2_launch video_streaming.launch.py \
  video_base_host:=192.168.1.50
```

Or enable it through the main real-rover launch:

```bash
cd ~/mr2-stack/rover/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mr2_launch rover_real.launch.py \
  enable_video_streaming:=true \
  video_base_host:=192.168.1.50
```

The node creates one GStreamer H.264 RTP pipeline per configured stream and
normalizes supported ROS images to RGB before encoding. Set
`encoder.type` to `nvv4l2h264enc` for Jetson hardware encoding, or `x264`
for the CPU encoder fallback.

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
