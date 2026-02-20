#!/usr/bin/env bash
set -euo pipefail

# -------- config (EDIT THESE) --------
# Where your repo root is (so relative cd works)
REPO_ROOT="${REPO_ROOT:-$HOME/mr2-stack}"

# ROS 2 environment(s) to source
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
WS_SETUP="${WS_SETUP:-$REPO_ROOT/rover/ros2_ws/install/setup.bash}"
# -------------------------------------

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
exec ros2 launch mr2_launch rover_real.launch.py enable_manipulator_module:=false enable_sik_sim:=true
CMD
)

GATEWAY_CMD=$(
  cat <<'CMD'
set -e
cd "$REPO_ROOT/base/gateway"
exec npm start -- --device /tmp/sik_sim1 --baud 57600 --port 8081
CMD
)

# Export variables so bash -lc heredocs can see them
export REPO_ROOT ROS_SETUP WS_SETUP

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
