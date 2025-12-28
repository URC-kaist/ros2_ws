#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

echo "Building dashboard..."
cd "$repo_root/dashboard"
npm install
npm run build

sudo mkdir -p /var/www/mr2-dashboard
sudo rsync -a --delete "$repo_root/dashboard/dist/" /var/www/mr2-dashboard/

if ! command -v nginx >/dev/null 2>&1; then
  echo "Installing nginx..."
  sudo apt update
  sudo apt install -y nginx
fi

sudo cp "$repo_root/scripts/nginx/mr2-dashboard.conf" /etc/nginx/sites-available/mr2
sudo ln -sf /etc/nginx/sites-available/mr2 /etc/nginx/sites-enabled/mr2

if [ -e /etc/nginx/sites-enabled/default ]; then
  sudo rm /etc/nginx/sites-enabled/default
fi

sudo nginx -t
sudo systemctl reload nginx

echo "Dashboard deployed to /var/www/mr2-dashboard"
