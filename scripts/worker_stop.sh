#!/usr/bin/env bash
# worker_stop.sh — installed on worker as /opt/topicfs/stop.sh.
# Copyright 2026 Jack Sidman Smith
# Licensed under the MIT License. See LICENSE in project root.
#
# Stops TopicFS. /mnt/topicfs stays a shared bind mount, ready for start.sh.
set -euo pipefail

sudo podman stop --ignore topicfs_runtime
