#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RULES_SRC="${WORKSPACE_ROOT}/src/mr2_launch/config/99-mr2-serial.rules"
RULES_DST="/etc/udev/rules.d/99-mr2-serial.rules"

if [[ ! -f "${RULES_SRC}" ]]; then
  echo "Missing rules file: ${RULES_SRC}" >&2
  exit 1
fi

sudo install -m 0644 "${RULES_SRC}" "${RULES_DST}"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Installed ${RULES_DST}"
echo "Reconnect USB serial devices or verify with: ls -l /dev/ttyXBEE"
