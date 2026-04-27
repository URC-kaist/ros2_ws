# Rover Gateway Startup

`rover/ros2_ws/scripts/start_rover_gateway.bash` starts the hardware rover
stack and the base gateway in detached `screen` sessions.

## What It Starts

- Configures `can0` by running `scripts/can0.bash` from the repository root.
- Starts `ros2 launch mr2_launch rover_real.launch.py` in a `screen` session
  named `ros_launch`.
- Starts the Node.js gateway in `base/gateway` in a `screen` session named
  `gateway`.
- Restarts nginx so the dashboard proxy routes are active.

The rover launch currently uses:

```bash
enable_manipulator_module:=true
enable_autonomous_module:=false
enable_xbee_sim:=true
use_servo:=true
```

The gateway currently connects to the emulated XBEE peer:

```bash
/tmp/xbee_sim1
```

The gateway targets the base Rocket M2 management address from the required
environment variable:

```bash
MR2_BASE_ROCKET_IP
```

## Prerequisites

- ROS 2 Humble is installed.
- `rover/ros2_ws` has been built and `install/setup.bash` exists.
- Node.js dependencies have been installed in `base/gateway`.
- `screen`, `sudo`, nginx, and CAN tooling are available on the host.

## Usage

From the repository root:

```bash
./rover/ros2_ws/scripts/start_rover_gateway.bash
```

Attach to the running sessions:

```bash
screen -r ros_launch
screen -r gateway
```

Detach from a session with `Ctrl+A`, then `D`.

List sessions:

```bash
screen -ls
```

## Overrides

The script can be pointed at non-default locations with environment variables:

```bash
REPO_ROOT=/home/mr2/mr2-stack \
ROS_SETUP=/opt/ros/humble/setup.bash \
WS_SETUP=/home/mr2/mr2-stack/rover/ros2_ws/install/setup.bash \
./rover/ros2_ws/scripts/start_rover_gateway.bash
```

Defaults:

- `REPO_ROOT=$HOME/mr2-stack`
- `ROS_SETUP=/opt/ros/humble/setup.bash`
- `WS_SETUP=$REPO_ROOT/rover/ros2_ws/install/setup.bash`

Network addresses are loaded from the repository root `.env` and `.env.local`
before the screen sessions start. Copy `.env.example` to `.env` and edit these
shared values when the base/rover network changes:

- `MR2_BASE_IP`
- `MR2_ROVER_IP`
- `MR2_BASE_ROCKET_IP`
- `MR2_DRONE_ROCKET_IP`
- `MR2_ROVER_ROCKET_IP`
- `MR2_GATEWAY_HOST`
- `MR2_GATEWAY_PORT`

## Behavior

If a `screen` session with the expected name already exists, the script leaves
it running and prints a message instead of starting a duplicate.

The script restarts nginx with:

```bash
sudo systemctl restart nginx
```
