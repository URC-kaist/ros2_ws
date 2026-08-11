#!/usr/bin/env bash
set -euo pipefail

mode="install"
if [[ "${1:-}" == "--check" ]]; then
  mode="check"
elif [[ $# -gt 0 ]]; then
  echo "Usage: $0 [--check]" >&2
  exit 2
fi

tcpdump_command="$(command -v tcpdump || true)"
if [[ -z "$tcpdump_command" ]]; then
  echo "tcpdump is not installed" >&2
  exit 1
fi
tcpdump_path="$(readlink -f "$tcpdump_command")"

echo "tcpdump: $tcpdump_path"
capabilities="$(getcap "$tcpdump_path" || true)"
printf '%s\n' "$capabilities"

if [[ "$mode" == "check" ]]; then
  if [[ "$capabilities" == *cap_net_admin* && "$capabilities" == *cap_net_raw* ]]; then
    echo "Capture capability is configured"
    exit 0
  fi
  echo "Capture capability is not configured" >&2
  exit 1
fi

sudo setcap cap_net_raw,cap_net_admin=eip "$tcpdump_path"
getcap "$tcpdump_path"
