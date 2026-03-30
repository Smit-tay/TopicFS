#!/usr/bin/env bash
# MIT License
# TopicFS runtime restart script.
# Safely tears down any stale FUSE mount and starts a fresh runtime container.
# Run as root (sudo) — required for rootful podman and FUSE mount propagation.

set -euo pipefail

CONTAINER_NAME="topicfs_runtime"
MOUNT_POINT="/home/jack/fuse_mount"
IMAGE="topicfs_runtime:latest"
INSTALL_DIR="/home/jack/dev/smithjack.net/topicfs/install"
TYPESUPPORT_DIR="/opt/topicfs_typesupport"

# --- Tear down ---

if podman container exists "${CONTAINER_NAME}" 2>/dev/null; then
    echo "Stopping existing container: ${CONTAINER_NAME}"
    podman stop "${CONTAINER_NAME}" 2>/dev/null || true
    podman rm   "${CONTAINER_NAME}" 2>/dev/null || true
fi

# Lazy unmount clears stale FUSE dentries regardless of mount state.
# -u = unmount, -z = lazy (detach immediately, clean up when refs drop).
# fusermount3 exits 0 if already unmounted — safe to call unconditionally.
fusermount3 -uz "${MOUNT_POINT}" 2>/dev/null || true

# Brief pause — let the kernel finish cleaning up the mount namespace entry.
sleep 1

# --- Start ---

echo "Starting ${CONTAINER_NAME}"
podman run -d \
    --name "${CONTAINER_NAME}" \
    --network host \
    --privileged \
    --device /dev/fuse \
    -v "${INSTALL_DIR}:${INSTALL_DIR}:z" \
    -v "${MOUNT_POINT}:${MOUNT_POINT}:shared" \
    -v "${TYPESUPPORT_DIR}:${TYPESUPPORT_DIR}:z" \
    -e ROS_DOMAIN_ID=11 \
    -e TOPICFS_MOUNT_POINT="${MOUNT_POINT}" \
    "${IMAGE}"

echo "Done. Tailing logs (Ctrl-C to exit):"
podman logs -f "${CONTAINER_NAME}"
