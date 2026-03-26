#!/bin/bash
# entrypoint.sh
# Copyright 2025 Jack Sidman Smith
# Licensed under the MIT License. See LICENSE in project root.
#
# Runtime entrypoint for the TopicFS container.

set -e

source /opt/ros/jazzy/setup.bash

if [ ! -f /home/jack/dev/smithjack.net/topicfs/install/setup.bash ]; then
    echo "ERROR: TopicFS install not found." >&2
    echo "  Run NF_00 (colcon build) first." >&2
    exit 1
fi

source /home/jack/dev/smithjack.net/topicfs/install/setup.bash

# Source any third-party typesupport packages
if [ -d /opt/topicfs_typesupport ]; then
    for setup in /opt/topicfs_typesupport/*/setup.bash; do
        if [ -f "$setup" ]; then
            echo "[topicfs] sourcing typesupport: $setup"
            source "$setup"
        fi
    done
fi

exec ros2 run topic_fs topic_fs \
    --ros-args \
    -p mount_point:=${TOPICFS_MOUNT_POINT:-/home/jack/fuse_mount}
