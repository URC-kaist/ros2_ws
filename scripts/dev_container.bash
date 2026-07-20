#!/usr/bin/env bash

set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
compose_file="$repo_root/docker/dev/compose.yaml"

export MR2_DEV_UID="${MR2_DEV_UID:-$(id -u)}"
export MR2_DEV_GID="${MR2_DEV_GID:-$(id -g)}"

if ! command -v docker >/dev/null 2>&1; then
  echo "Docker is not installed or is not available in PATH." >&2
  exit 1
fi

compose=(docker compose --file "$compose_file")

usage() {
  echo "Usage: $0 {build|up|setup|shell|check|logs|down}" >&2
}

command_name="${1:-}"
case "$command_name" in
  build)
    "${compose[@]}" build
    ;;
  up)
    "${compose[@]}" up --detach workspace
    ;;
  setup)
    "${compose[@]}" exec workspace \
      /usr/local/bin/mr2-dev-entrypoint bash docker/dev/setup_workspace.bash
    ;;
  shell)
    "${compose[@]}" exec workspace /usr/local/bin/mr2-dev-entrypoint bash
    ;;
  check)
    "${compose[@]}" exec workspace \
      /usr/local/bin/mr2-dev-entrypoint bash docker/dev/check.bash
    ;;
  logs)
    "${compose[@]}" logs --follow workspace
    ;;
  down)
    "${compose[@]}" down
    ;;
  *)
    usage
    exit 2
    ;;
esac
