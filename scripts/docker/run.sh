#!/usr/bin/env bash
set -euo pipefail

# ----------------------------
# Config you might edit
# ----------------------------
IMAGE_NAME="${IMAGE_NAME:-tr-robot}"
CONTAINER_NAME="${CONTAINER_NAME:-tr-robot}"
HOSTNAME_IN_CONTAINER="${HOSTNAME_IN_CONTAINER:-rover}"

# Persist Transitive state on the host (this is the important one)
STATE_DIR="${STATE_DIR:-$HOME/.tr_docker}"

# Unique per robot. Change if you clone images across multiple robots.
TR_INSTALL_HASH="${TR_INSTALL_HASH:-robot1}"

# Optional ROS envs
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# ----------------------------
# Build image (if needed)
# ----------------------------
echo "[*] Ensuring state dir exists: $STATE_DIR"
mkdir -p "$STATE_DIR"

if ! sudo docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
  echo "[*] Image '$IMAGE_NAME' not found. Building..."
  sudo docker build -t "$IMAGE_NAME" .
fi

# ----------------------------
# Run container
# ----------------------------
echo "[*] Starting container '$CONTAINER_NAME' from image '$IMAGE_NAME'..."

sudo docker run -d \
  --restart unless-stopped \
  --privileged \
  --name "$CONTAINER_NAME" \
  --network=host \
  --hostname "$HOSTNAME_IN_CONTAINER" \
  -v /dev/videoTOP:/dev/videoTOP \
  -v "$STATE_DIR:/root/.transitive" \
  -v /run/udev:/run/udev:ro \
  -e TR_INSTALL_HASH="$TR_INSTALL_HASH" \
  -e ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  -e RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
  "$IMAGE_NAME"
