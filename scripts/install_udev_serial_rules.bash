#!/usr/bin/env bash
set -euo pipefail

RULES_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/99-mr2-serial.rules"
RULES_DST="/etc/udev/rules.d/99-mr2-serial.rules"

if [[ ! -f "$RULES_SRC" ]]; then
  echo "Rules file not found: $RULES_SRC" >&2
  exit 1
fi

if [[ $EUID -ne 0 ]]; then
  if command -v sudo >/dev/null 2>&1; then
    SUDO="sudo"
  else
    echo "Error: sudo not found; run as root to write $RULES_DST." >&2
    exit 1
  fi
else
  SUDO=""
fi

$SUDO install -d "$(dirname "$RULES_DST")"
$SUDO install -m 0644 "$RULES_SRC" "$RULES_DST"
$SUDO udevadm control --reload-rules
$SUDO udevadm trigger

echo "Installed $RULES_DST"
