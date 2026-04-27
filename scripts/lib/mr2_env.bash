#!/usr/bin/env bash

mr2_repo_root() {
  cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd
}

mr2_load_env() {
  local repo_root="${1:-$(mr2_repo_root)}"
  local env_file

  set -a
  for env_file in "$repo_root/.env" "$repo_root/.env.local"; do
    if [ -f "$env_file" ]; then
      # shellcheck disable=SC1090
      source "$env_file"
    fi
  done
  set +a

  export \
    MR2_BASE_IP \
    MR2_ROVER_IP \
    MR2_BASE_ROCKET_IP \
    MR2_DRONE_ROCKET_IP \
    MR2_ROVER_ROCKET_IP \
    MR2_GATEWAY_HOST \
    MR2_GATEWAY_PORT \
    MR2_ROSBRIDGE_PORT
}

mr2_require_env() {
  local name

  for name in "$@"; do
    if [ -z "${!name:-}" ]; then
      echo "Missing required environment variable: $name" >&2
      return 1
    fi
  done
}
