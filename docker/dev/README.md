# MR2 laptop development container

This environment is for developing and validating the MR2 stack on an Ubuntu
24.04 laptop. It runs ROS 2 Humble in Ubuntu 22.04 (Jammy), along with Node.js
22, GStreamer, `socat`, SocketCAN tools, and the C/C++ build toolchain.

It is not the Jetson deployment environment. The rover continues to run its
ROS, gateway, nginx, GStreamer, and AP services natively.

## Prerequisites

Install Docker Engine and the Docker Compose plugin, then make sure the current
user can run `docker` without `sudo`.

Initialize the repository submodules on the host:

```bash
git submodule update --init --recursive
```

## First-time setup

From the repository root:

```bash
./scripts/dev_container.bash build
./scripts/dev_container.bash up
./scripts/dev_container.bash setup
```

The repository is bind-mounted at `/workspace/mr2-stack`. ROS build outputs and
`node_modules` therefore persist in the host checkout and remain covered by the
repository ignore rules. Setup builds the RTCM/u-blox interface packages before
installing `rclnodejs`, so the gateway bindings are generated against the
sourced Humble workspace.

## Shell and checks

Open an interactive shell:

```bash
./scripts/dev_container.bash shell
```

Run gateway tests, the complete dashboard check, and the ROS direct-control
package build/tests:

```bash
./scripts/dev_container.bash check
```

Stop the development container:

```bash
./scripts/dev_container.bash down
```

## Manual XBEE simulation

The current simulation launch creates `/tmp/xbee_sim0` for the rover bridge and
`/tmp/xbee_sim1` for the gateway. Use separate container shells.

Terminal 1:

```bash
cd /workspace/mr2-stack/rover/ros2_ws
source install/setup.bash
ros2 launch mr2_launch rover_sim.launch.py headless:=true
```

Terminal 2:

```bash
cd /workspace/mr2-stack/base/gateway
npm start -- \
  --gateway-profile rover-direct \
  --base-xbee-device /tmp/xbee_sim1 \
  --gateway-host 0.0.0.0 \
  --gateway-port 8081
```

Terminal 3:

```bash
cd /workspace/mr2-stack/dashboard
VITE_OPERATING_PROFILE=rover-direct npm run dev
```

The dashboard is then available from the laptop at the URL printed by Vite.
The development container is not used by the Jetson deployment; see
[`docs/rover-direct-operation.md`](../../docs/rover-direct-operation.md) for
the native runtime.

## Video limitations

The image includes the CPU GStreamer plugins used by the gateway and test
pipelines. It does not include Jetson's `nvv4l2h264enc` or `nvvidconv`.
Laptop end-to-end video tests must use an `x264` or synthetic H.264 source.
The authoritative stream IDs and UDP ports remain in
`rover/ros2_ws/src/mr2_launch/config/video_streams.json`.

## Optional dependencies

`ultralytics` is intentionally not installed because direct manual control
does not run YOLO. Install it inside the container only when working on that
package:

```bash
python3 -m pip install --user ultralytics
```
