#!/usr/bin/env bash
set -euo pipefail

# Start the rover ROS launch and base gateway in detached screen sessions.
# Override these with environment variables when the repo or ROS install lives
# somewhere else:
#   REPO_ROOT=/path/to/mr2-stack
#   ROS_SETUP=/opt/ros/humble/setup.bash
#   WS_SETUP=/path/to/ros2_ws/install/setup.bash
REPO_ROOT="${REPO_ROOT:-$HOME/mr2-stack}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-$REPO_ROOT/rover/ros2_ws/install/setup.bash}"

# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/lib/mr2_env.bash"
mr2_load_env "$REPO_ROOT"
mr2_require_env MR2_BASE_IP MR2_GATEWAY_HOST MR2_GATEWAY_PORT MR2_BASE_ROCKET_IP MR2_DRONE_ROCKET_IP MR2_ROVER_ROCKET_IP

need_cmd() { command -v "$1" >/dev/null 2>&1 || { echo "Missing command: $1" >&2; exit 1; }; }
need_cmd screen
need_cmd sudo

cd "$REPO_ROOT"

./scripts/can0.bash

start_screen_if_missing() {
  local name="$1"
  local cmd="$2"

  if screen -list | grep -qE "[[:space:]]${name}[[:space:]]"; then
    echo "[screen] session already exists: $name"
    return 0
  fi

  echo "[screen] starting: $name"
  # Run a login shell so "source" works; keep the session open after the command ends.
  screen -dmS "$name" bash -lc "$cmd; echo; echo '[screen] $name exited (press Ctrl+A then D to detach)'; exec bash"
}

ROS_CMD=$(
  cat <<'CMD'
set -e
source "$ROS_SETUP"
source "$WS_SETUP"
exec ros2 launch mr2_launch rover_real.launch.py enable_manipulator_module:=true enable_autonomous_module:=false enable_xbee_sim:=true use_servo:=true video_base_host:="$MR2_BASE_IP"
CMD
)

GATEWAY_CMD=$(
  cat <<'CMD'
set -e
cd "$REPO_ROOT/base/gateway"
exec npm start -- --base-xbee-device /tmp/xbee_sim1 --gateway-host "$MR2_GATEWAY_HOST" --gateway-port "$MR2_GATEWAY_PORT" --base-rocket-m2-ip "$MR2_BASE_ROCKET_IP" --drone-rocket-m2-ip "$MR2_DRONE_ROCKET_IP" --rover-rocket-m2-ip "$MR2_ROVER_ROCKET_IP"
CMD
)

# Export variables so bash -lc heredocs can see them
export REPO_ROOT ROS_SETUP WS_SETUP MR2_BASE_IP MR2_GATEWAY_HOST MR2_GATEWAY_PORT MR2_BASE_ROCKET_IP MR2_DRONE_ROCKET_IP MR2_ROVER_ROCKET_IP

start_screen_if_missing "ros_launch" "$ROS_CMD"
start_screen_if_missing "gateway"    "$GATEWAY_CMD"

echo "[systemd] restarting nginx"
sudo systemctl restart nginx

echo
echo "Done."
echo "Attach to sessions:"
echo "  screen -r ros_launch"
echo "  screen -r gateway"
echo "List sessions:"
echo "  screen -ls"
