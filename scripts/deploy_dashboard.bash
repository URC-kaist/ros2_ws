#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

echo "Building dashboard..."
cd "$repo_root/dashboard"
npm install
npm run build

sudo mkdir -p /var/www/mr2-dashboard
sudo rsync -a --delete "$repo_root/dashboard/dist/" /var/www/mr2-dashboard/

echo "Dashboard deployed to /var/www/mr2-dashboard"
