#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
DOCKERFILE_PATH="$SCRIPT_DIR/dockerfile"

# ----------------------------
# Config you might edit
# ----------------------------
IMAGE_NAME="${IMAGE_NAME:-tr-robot}"
CONTAINER_NAME="${CONTAINER_NAME:-tr-robot}"
HOSTNAME_IN_CONTAINER="${HOSTNAME_IN_CONTAINER:-rover}"

# Persist Transitive state on the host
STATE_DIR="${STATE_DIR:-$HOME/.tr_docker}"

# Unique per robot. Change if you clone images across multiple robots.
TR_INSTALL_HASH="${TR_INSTALL_HASH:-robot1}"

# Optional ROS envs
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"

# ----------------------------
# Run-as-user integration (host user -> container)
# ----------------------------
HOST_USER="${HOST_USER:-mr2}"
HOST_UID="${HOST_UID:-$(id -u)}"
HOST_GID="${HOST_GID:-$(id -g)}"

CONTAINER_HOME="${CONTAINER_HOME:-/home/$HOST_USER}"
STATE_DIR_IN_CONTAINER="${STATE_DIR_IN_CONTAINER:-$CONTAINER_HOME/.transitive}"

# ----------------------------
# Build image (if needed)
# ----------------------------
echo "[*] Ensuring state dir exists: $STATE_DIR"
mkdir -p "$STATE_DIR"

if ! sudo docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
  echo "[*] Image '$IMAGE_NAME' not found. Building..."
  sudo docker build -t "$IMAGE_NAME" -f "$DOCKERFILE_PATH" "$REPO_ROOT"
fi

# ----------------------------
# Run container
# ----------------------------
echo "[*] Starting container '$CONTAINER_NAME' from image '$IMAGE_NAME'..."
if sudo docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
  echo "[*] Container '$CONTAINER_NAME' already exists. Restarting..."
  sudo docker restart "$CONTAINER_NAME" >/dev/null
  echo "[*] Done."
  exit 0
fi

sudo docker run -d \
  --restart unless-stopped \
  --privileged \
  --name "$CONTAINER_NAME" \
  --net=host \
  --ipc=host \
  --pid=host \
  --hostname "$HOSTNAME_IN_CONTAINER" \
  --user "$HOST_UID:$HOST_GID" \
  -v /etc/passwd:/etc/passwd:ro \
  -v /etc/group:/etc/group:ro \
  -e HOME="$CONTAINER_HOME" \
  -v "$STATE_DIR:$STATE_DIR_IN_CONTAINER" \
  -v /run/udev:/run/udev:ro \
  -e TR_INSTALL_HASH="$TR_INSTALL_HASH" \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
  -e ROS_LOCALHOST_ONLY="$ROS_LOCALHOST_ONLY" \
  "$IMAGE_NAME"

echo "[*] Container '$CONTAINER_NAME' started."
echo "    - user: $HOST_USER ($HOST_UID:$HOST_GID)"
echo "    - home: $CONTAINER_HOME"
echo "    - transitive state: $STATE_DIR -> $STATE_DIR_IN_CONTAINER"
