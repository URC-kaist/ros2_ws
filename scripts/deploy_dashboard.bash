#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$repo_root/scripts/lib/mr2_env.bash"
mr2_load_env "$repo_root"
mr2_require_env \
  MR2_BASE_IP \
  MR2_ROVER_IP \
  MR2_BASE_ROCKET_IP \
  MR2_DRONE_ROCKET_IP \
  MR2_ROVER_ROCKET_IP \
  MR2_GATEWAY_HOST \
  MR2_GATEWAY_PORT \
  MR2_ROSBRIDGE_PORT

ssl_dir=/etc/nginx/ssl
ssl_key="$ssl_dir/mr2-dashboard.key"
ssl_cert="$ssl_dir/mr2-dashboard.crt"
nginx_snippet=/etc/nginx/snippets/mr2-dashboard-locations.conf
nginx_conf=$(mktemp)

cleanup() {
  rm -f "$nginx_conf"
}
trap cleanup EXIT

render_nginx_conf() {
  sed \
    -e "s/__MR2_GATEWAY_HOST__/$MR2_GATEWAY_HOST/g" \
    -e "s/__MR2_GATEWAY_PORT__/$MR2_GATEWAY_PORT/g" \
    -e "s/__MR2_ROVER_IP__/$MR2_ROVER_IP/g" \
    -e "s/__MR2_ROSBRIDGE_PORT__/$MR2_ROSBRIDGE_PORT/g" \
    -e "s/__MR2_BASE_IP__/$MR2_BASE_IP/g" \
    -e "s/__MR2_BASE_ROCKET_IP__/$MR2_BASE_ROCKET_IP/g" \
    -e "s/__MR2_DRONE_ROCKET_IP__/$MR2_DRONE_ROCKET_IP/g" \
    -e "s/__MR2_ROVER_ROCKET_IP__/$MR2_ROVER_ROCKET_IP/g" \
    "$repo_root/scripts/nginx/mr2-dashboard.conf.template" > "$nginx_conf"
}

echo "Building dashboard..."
cd "$repo_root/dashboard"
npm install
npm run build

sudo mkdir -p /var/www/mr2-dashboard
sudo mkdir -p /var/www/mr2-tiles
sudo rsync -a --delete "$repo_root/dashboard/dist/" /var/www/mr2-dashboard/

if ! command -v nginx >/dev/null 2>&1; then
  echo "Installing nginx..."
  sudo apt update
  sudo apt install -y nginx
fi

if [ ! -f "$ssl_key" ] || [ ! -f "$ssl_cert" ]; then
  echo "Generating self-signed dashboard TLS certificate..."
  sudo mkdir -p "$ssl_dir"
  sudo openssl req -x509 -nodes -days 365 \
    -newkey rsa:2048 \
    -keyout "$ssl_key" \
    -out "$ssl_cert" \
    -subj "/CN=$MR2_BASE_IP" \
    -addext "subjectAltName=IP:$MR2_BASE_IP,DNS:base,DNS:base.local"
  sudo chmod 0600 "$ssl_key"
  sudo chmod 0644 "$ssl_cert"
fi

render_nginx_conf
sudo mkdir -p "$(dirname "$nginx_snippet")"
sudo cp "$repo_root/scripts/nginx/mr2-dashboard-locations.conf" "$nginx_snippet"
sudo cp "$nginx_conf" /etc/nginx/sites-available/mr2
sudo ln -sf /etc/nginx/sites-available/mr2 /etc/nginx/sites-enabled/mr2

if [ -e /etc/nginx/sites-enabled/default ]; then
  sudo rm /etc/nginx/sites-enabled/default
fi

sudo nginx -t
sudo systemctl reload nginx

echo "Dashboard deployed to /var/www/mr2-dashboard"
