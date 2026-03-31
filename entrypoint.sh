#!/bin/bash
# entrypoint.sh
# Copyright 2025 Jack Sidman Smith
# Licensed under the MIT License. See LICENSE in project root.
#
# Runtime entrypoint for the TopicFS container.
# Environment variables:
#   TOPICFS_INSTALL     — path to colcon install tree
#   TOPICFS_MOUNT_POINT — FUSE mount point

source /opt/ros/jazzy/setup.bash

TOPICFS_INSTALL=${TOPICFS_INSTALL:-${HOME}/dev/smithjack.net/topicfs/install}
MOUNT_POINT=${TOPICFS_MOUNT_POINT:-${HOME}/fuse_mount}

if [ ! -f "${TOPICFS_INSTALL}/setup.bash" ]; then
    echo "ERROR: TopicFS install not found at ${TOPICFS_INSTALL}" >&2
    echo "  Set TOPICFS_INSTALL or run NF_00 (colcon build) first." >&2
    exit 1
fi

source "${TOPICFS_INSTALL}/setup.bash"

# Source any third-party typesupport packages
if [ -d /opt/topicfs_typesupport ]; then
    for setup in /opt/topicfs_typesupport/*/setup.bash; do
        if [ -f "$setup" ]; then
            echo "[topicfs] sourcing typesupport: $setup"
            source "$setup"
        fi
    done
fi

cleanup() {
    echo "[topicfs] unmounting ${MOUNT_POINT}"
    fusermount3 -u "${MOUNT_POINT}" 2>/dev/null || true
}

term_handler() {
    if [ -n "${NODE_PID}" ]; then
        kill -TERM "${NODE_PID}" 2>/dev/null
        wait "${NODE_PID}"
    fi
    cleanup
}

trap term_handler TERM
trap cleanup EXIT

NODE_PID=""

ros2 run topic_fs topic_fs \
    --ros-args \
    -p mount_point:="${MOUNT_POINT}" &

NODE_PID=$!
wait "${NODE_PID}"
