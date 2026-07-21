#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
workspace="$repo_root/rover/ros2_ws"
gateway_entrypoint="$repo_root/base/gateway/index.js"
node_bin=""

# Prefer the repository-pinned NVM runtime. This also protects systemd starts
# from an older installed unit that still contains a stale MR2_NODE_BIN path.
if [[ -f $repo_root/.nvmrc ]]; then
  IFS= read -r pinned_node_version <"$repo_root/.nvmrc" || true
  pinned_node_version=${pinned_node_version#v}
  nvm_dir=${NVM_DIR:-${HOME:-}/.nvm}
  pinned_node="$nvm_dir/versions/node/v$pinned_node_version/bin/node"
  if [[ -x $pinned_node ]]; then
    node_bin=$pinned_node
  fi
fi
node_bin=${node_bin:-${MR2_NODE_BIN:-$(command -v node || true)}}

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is not installed at /opt/ros/humble." >&2
  exit 1
fi
if [[ ! -f "$workspace/install/setup.bash" ]]; then
  echo "Build the rover workspace before starting direct mode: $workspace" >&2
  exit 1
fi
if [[ -z $node_bin || ! -x $node_bin ]]; then
  echo "Node.js is not available. Install/select Node.js 24 first." >&2
  exit 1
fi
node_version=$($node_bin --version)
node_major=$($node_bin -p 'process.versions.node.split(".")[0]')
if [[ $node_major != 24 ]]; then
  echo "Node.js 24 is required; found $node_version at $node_bin." >&2
  exit 1
fi
if [[ ! -f $gateway_entrypoint ]]; then
  echo "Gateway entrypoint not found: $gateway_entrypoint" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$workspace/install/setup.bash"
set -u

exec ros2 launch mr2_launch rover_direct.launch.py \
  can_iface:="${MR2_CAN_IFACE:-can0}" \
  controller_spawn_delay:="${MR2_CONTROLLER_SPAWN_DELAY:-10.0}" \
  controller_config:="${MR2_CONTROLLER_CONFIG:-$workspace/install/mr2_rover_description/share/mr2_rover_description/config/controllers/rover_controllers.yaml}" \
  headless:="${MR2_HEADLESS:-true}" \
  rviz_config:="${MR2_RVIZ_CONFIG:-$workspace/install/mr2_launch/share/mr2_launch/rviz/sim.rviz}" \
  use_mock_servos:="${MR2_USE_MOCK_SERVOS:-false}" \
  enable_manipulator_module:="${MR2_ENABLE_MANIPULATOR_MODULE:-true}" \
  start_manipulator_controllers_active:="${MR2_START_MANIPULATOR_CONTROLLERS_ACTIVE:-true}" \
  enable_autonomous_module:="${MR2_ENABLE_AUTONOMOUS_MODULE:-false}" \
  enable_aruco:="${MR2_ENABLE_ARUCO:-false}" \
  aruco_cam_topic:="${MR2_ARUCO_CAM_TOPIC:-/front_camera/image_raw}" \
  enable_video_streaming:="${MR2_ENABLE_VIDEO_STREAMING:-true}" \
  video_config:="${MR2_VIDEO_CONFIG:-$workspace/install/mr2_launch/share/mr2_launch/config/video_streams.json}" \
  yolo_cam_topic:="${MR2_YOLO_CAM_TOPIC:-/rgbd_camera}" \
  yolo_device:="${MR2_YOLO_DEVICE:-cuda:0}" \
  yolo_publish_annotated:="${MR2_YOLO_PUBLISH_ANNOTATED:-true}" \
  yolo_annotated_fps:="${MR2_YOLO_ANNOTATED_FPS:-0.5}" \
  enable_led:="${MR2_ENABLE_LED:-false}" \
  led_can_id:="${MR2_LED_CAN_ID:-0x123}" \
  enable_camera_turret:="${MR2_ENABLE_CAMERA_TURRET:-false}" \
  camera_turret_can_id:="${MR2_CAMERA_TURRET_CAN_ID:-0x124}" \
  camera_turret_invert_x:="${MR2_CAMERA_TURRET_INVERT_X:-false}" \
  camera_turret_invert_y:="${MR2_CAMERA_TURRET_INVERT_Y:-false}" \
  xbee_device:="${MR2_ROVER_XBEE_DEVICE:-/tmp/mr2_xbee_rover}" \
  xbee_gateway_device:="${MR2_GATEWAY_XBEE_DEVICE:-/tmp/mr2_xbee_gateway}" \
  start_xbee_sim:="${MR2_START_XBEE_SIM:-true}" \
  start_gateway:="${MR2_START_GATEWAY:-true}" \
  ensure_nginx:="${MR2_ENSURE_NGINX:-true}" \
  start_rover:="${MR2_START_ROVER:-true}" \
  node_binary:="$node_bin" \
  gateway_entrypoint:="$gateway_entrypoint" \
  gateway_host:="${MR2_GATEWAY_HOST:-127.0.0.1}" \
  gateway_port:="${MR2_GATEWAY_PORT:-8081}" \
  video_base_host:=127.0.0.1
