#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
run_user=${SUDO_USER:-$USER}
run_group=$(id -gn "$run_user")
direct_ip=${MR2_DIRECT_AP_IP:-192.168.2.102}
enable_services=false
start_services=false
manual_launch=true
skip_build=false

usage() {
  cat <<'EOF'
Usage: scripts/install_rover_direct.bash [--skip-build] [--manual|--enable|--start]

Installs the native Jetson dashboard/nginx and systemd unit files.
  --skip-build  Reuse installed gateway dependencies and dashboard/dist
  --manual      Stop/disable the rover service for direct ros2 launch (default)
  --enable      Use systemd mode: enable rover-direct at boot without starting
  --start       Use systemd mode: enable and start/restart rover-direct

The ROS workspace must already contain rover/ros2_ws/install/setup.bash.
Configure the Wi-Fi AP separately with scripts/configure_rover_ap.bash.
EOF
}

while (($# > 0)); do
  case "$1" in
    --skip-build)
      skip_build=true
      ;;
    --manual)
      manual_launch=true
      enable_services=false
      start_services=false
      ;;
    --enable)
      manual_launch=false
      enable_services=true
      ;;
    --start)
      manual_launch=false
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

# Resolve the repository-pinned NVM runtime automatically, even when this
# installer is invoked from an older shell whose PATH still points at Node 22.
if [[ -f $repo_root/.nvmrc ]]; then
  IFS= read -r pinned_node_version <"$repo_root/.nvmrc" || true
  pinned_node_version=${pinned_node_version#v}
  nvm_dir=${NVM_DIR:-${HOME:-}/.nvm}
  pinned_node_bin_dir="$nvm_dir/versions/node/v$pinned_node_version/bin"
  if [[ -x $pinned_node_bin_dir/node ]]; then
    export PATH="$pinned_node_bin_dir:$PATH"
  fi
fi

required_commands=(node npm nginx openssl rsync sed socat)
for command_name in "${required_commands[@]}"; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "Required native dependency is missing: $command_name" >&2
    exit 1
  fi
done

node_bin=$(command -v node)
node_version=$($node_bin --version)
node_major=$($node_bin -p 'process.versions.node.split(".")[0]')
if [[ $node_major != 24 ]]; then
  echo "Node.js 24 is required; found $node_version at $node_bin." >&2
  echo "Install the version pinned in $repo_root/.nvmrc, then retry." >&2
  exit 1
fi
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

for unit in mr2-rover-direct; do
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
certificate_ips=(192.168.0.62 10.42.0.1)
if [[ " ${certificate_ips[*]} " != *" $direct_ip "* ]]; then
  certificate_ips+=("$direct_ip")
fi
certificate_sans=()
for certificate_ip in "${certificate_ips[@]}"; do
  certificate_sans+=("IP:$certificate_ip")
done
certificate_sans+=(DNS:rover DNS:rover.local)
certificate_san_csv=$(IFS=,; echo "${certificate_sans[*]}")

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

certificate_needs_refresh=false
if [[ ! -f $ssl_key || ! -f $ssl_cert ]]; then
  certificate_needs_refresh=true
else
  current_sans=$(openssl x509 -in "$ssl_cert" -noout -ext subjectAltName 2>/dev/null || true)
  for certificate_ip in "${certificate_ips[@]}"; do
    if [[ $current_sans != *"IP Address:$certificate_ip"* ]]; then
      certificate_needs_refresh=true
      break
    fi
  done
fi

if [[ $certificate_needs_refresh == true ]]; then
  echo "Generating rover-direct TLS certificate for: ${certificate_ips[*]}"
  generated_key="$rendered_dir/mr2-rover-direct.key"
  generated_cert="$rendered_dir/mr2-rover-direct.crt"
  (
    umask 077
    openssl req -x509 -nodes -days 365 \
      -newkey rsa:2048 \
      -keyout "$generated_key" \
      -out "$generated_cert" \
      -subj "/CN=$direct_ip" \
      -addext "subjectAltName=$certificate_san_csv"
  )
  sudo install -m 0600 "$generated_key" "$ssl_key"
  sudo install -m 0644 "$generated_cert" "$ssl_cert"
fi

for unit in mr2-rover-direct; do
  sudo install -m 0644 \
    "$rendered_dir/${unit}.service" \
    "/etc/systemd/system/${unit}.service"
done

sudo nginx -t
sudo systemctl daemon-reload
sudo systemctl enable --now nginx
sudo systemctl reload nginx

# Older rover-direct installs used separate PTY and gateway services. Keep them
# disabled so they cannot race the processes now owned by rover_direct.launch.py.
legacy_services=(mr2-xbee-sim.service mr2-gateway-direct.service)
sudo systemctl disable --now "${legacy_services[@]}" 2>/dev/null || true

services=(mr2-rover-direct.service)
if [[ $manual_launch == true ]]; then
  sudo systemctl disable --now "${services[@]}" 2>/dev/null || true
elif [[ $enable_services == true ]]; then
  sudo systemctl enable "${services[@]}"
fi
if [[ $start_services == true ]]; then
  sudo systemctl restart "${services[@]}"
fi

echo "Installed rover-direct files for https://${direct_ip}"
echo "TLS certificate IP SANs: ${certificate_ips[*]}"
echo "AP configuration was not changed. Run scripts/configure_rover_ap.bash separately."
if [[ $manual_launch == true ]]; then
  echo "Manual launch mode is ready; mr2-rover-direct.service is stopped and disabled."
  echo "Source the ROS workspace and run: ros2 launch mr2_launch rover_direct.launch.py"
elif [[ $start_services == false ]]; then
  echo "The rover-direct service was enabled but not started."
fi
