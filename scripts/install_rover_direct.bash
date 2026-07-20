#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_user=${SUDO_USER:-$USER}
run_group=$(id -gn "$run_user")
direct_ip=${MR2_DIRECT_AP_IP:-192.168.2.102}
enable_services=false
start_services=false
skip_build=false

usage() {
  cat <<'EOF'
Usage: scripts/install_rover_direct.bash [--skip-build] [--enable] [--start]

Installs the native Jetson dashboard/nginx and systemd unit files.
  --skip-build  Reuse installed gateway dependencies and dashboard/dist
  --enable      Enable the three MR2 services at boot
  --start       Enable and start/restart MR2 services and reload nginx

The ROS workspace must already contain rover/ros2_ws/install/setup.bash.
Configure the Wi-Fi AP separately with scripts/configure_rover_ap.bash.
EOF
}

while (($# > 0)); do
  case "$1" in
    --skip-build)
      skip_build=true
      ;;
    --enable)
      enable_services=true
      ;;
    --start)
      enable_services=true
      start_services=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

required_commands=(node npm nginx openssl rsync sed socat)
for command_name in "${required_commands[@]}"; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "Required native dependency is missing: $command_name" >&2
    exit 1
  fi
done

node_bin=$(command -v node)
ros_setup=/opt/ros/humble/setup.bash
ros_overlay="$repo_root/rover/ros2_ws/install/setup.bash"
if [[ ! -f $ros_setup ]]; then
  echo "ROS 2 Humble is not installed at $ros_setup." >&2
  exit 1
fi
if [[ ! -f $ros_overlay ]]; then
  echo "Missing ROS workspace overlay: $ros_overlay" >&2
  echo "Build rover/ros2_ws with ROS 2 Humble before installing direct mode." >&2
  exit 1
fi

if [[ $skip_build == false ]]; then
  (
    cd "$repo_root/base/gateway"
    set +u
    # shellcheck disable=SC1091
    source "$ros_setup"
    # shellcheck disable=SC1090
    source "$ros_overlay"
    set -u
    npm ci
  )
  (
    cd "$repo_root/dashboard"
    npm ci --include=dev
    VITE_OPERATING_PROFILE=rover-direct \
      VITE_ROSBRIDGE_URL= \
      VITE_XBEE_WS_URL= \
      VITE_VIDEO_WS_URL= \
      VITE_VIDEO_STREAMS_URL= \
      npm run build
  )
elif [[ ! -d $repo_root/dashboard/dist ]]; then
  echo "--skip-build was used but dashboard/dist does not exist." >&2
  exit 1
fi

escape_replacement() {
  sed 's/[&|]/\\&/g' <<<"$1"
}

escaped_repo_root=$(escape_replacement "$repo_root")
escaped_user=$(escape_replacement "$run_user")
escaped_group=$(escape_replacement "$run_group")
escaped_node=$(escape_replacement "$node_bin")

rendered_dir=$(mktemp -d)
cleanup() {
  rm -rf "$rendered_dir"
}
trap cleanup EXIT

sed "s|__MR2_DIRECT_AP_IP__|$(escape_replacement "$direct_ip")|g" \
  "$repo_root/scripts/nginx/mr2-rover-direct.conf.template" \
  >"$rendered_dir/mr2-rover-direct.conf"

for unit in mr2-xbee-sim mr2-gateway-direct mr2-rover-direct; do
  sed \
    -e "s|__REPO_ROOT__|$escaped_repo_root|g" \
    -e "s|__MR2_USER__|$escaped_user|g" \
    -e "s|__MR2_GROUP__|$escaped_group|g" \
    -e "s|__NODE_BIN__|$escaped_node|g" \
    "$repo_root/scripts/systemd/${unit}.service.template" \
    >"$rendered_dir/${unit}.service"
done

ssl_dir=/etc/nginx/ssl
ssl_key="$ssl_dir/mr2-rover-direct.key"
ssl_cert="$ssl_dir/mr2-rover-direct.crt"

sudo install -d -m 0755 /var/www/mr2-dashboard
sudo rsync -a --delete "$repo_root/dashboard/dist/" /var/www/mr2-dashboard/
sudo install -d -m 0755 /etc/nginx/snippets "$ssl_dir"
sudo install -m 0644 \
  "$repo_root/scripts/nginx/mr2-rover-direct-locations.conf" \
  /etc/nginx/snippets/mr2-rover-direct-locations.conf
sudo install -m 0644 \
  "$rendered_dir/mr2-rover-direct.conf" \
  /etc/nginx/sites-available/mr2-rover-direct
sudo ln -sfn \
  /etc/nginx/sites-available/mr2-rover-direct \
  /etc/nginx/sites-enabled/mr2-rover-direct

if [[ ! -f $ssl_key || ! -f $ssl_cert ]]; then
  sudo openssl req -x509 -nodes -days 365 \
    -newkey rsa:2048 \
    -keyout "$ssl_key" \
    -out "$ssl_cert" \
    -subj "/CN=$direct_ip" \
    -addext "subjectAltName=IP:$direct_ip,DNS:rover,DNS:rover.local"
  sudo chmod 0600 "$ssl_key"
  sudo chmod 0644 "$ssl_cert"
fi

for unit in mr2-xbee-sim mr2-gateway-direct mr2-rover-direct; do
  sudo install -m 0644 \
    "$rendered_dir/${unit}.service" \
    "/etc/systemd/system/${unit}.service"
done

sudo nginx -t
sudo systemctl daemon-reload

services=(mr2-xbee-sim.service mr2-gateway-direct.service mr2-rover-direct.service)
if [[ $enable_services == true ]]; then
  sudo systemctl enable "${services[@]}"
fi
if [[ $start_services == true ]]; then
  sudo systemctl enable --now nginx
  sudo systemctl restart "${services[@]}"
  sudo systemctl reload nginx
fi

echo "Installed rover-direct files for https://${direct_ip}"
echo "AP configuration was not changed. Run scripts/configure_rover_ap.bash separately."
if [[ $start_services == false ]]; then
  echo "Services were not started. Re-run with --start after checking CAN and cameras."
fi
