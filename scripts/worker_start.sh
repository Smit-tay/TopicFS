#!/usr/bin/env bash
# worker_start.sh — installed on worker as /opt/topicfs/start.sh.
# Copyright 2026 Jack Sidman Smith
# Licensed under the MIT License. See LICENSE in project root.
#
# Clears any stale FUSE mount, prepares /mnt/topicfs and (re)creates the
# container. The bind mount itself comes from /etc/fstab at boot:
#   /mnt/topicfs /mnt/topicfs none bind,shared,nofail 0 0
set -euo pipefail

MOUNT=/mnt/topicfs
COMPOSE=/opt/topicfs/docker-compose.runtime.yml
export DOCKER_HOST=unix:///run/podman/podman.sock

# rm, not compose down: an older container of this name may not belong to compose.
sudo podman rm -f --ignore topicfs_runtime

# Each stale FUSE instance is one layer here; lift them all.
while findmnt -n "${MOUNT}" >/dev/null 2>&1; do
    sudo umount -l "${MOUNT}"
done
sudo mount --bind "${MOUNT}" "${MOUNT}"
sudo mount --make-shared "${MOUNT}"

sudo --preserve-env=DOCKER_HOST docker-compose -f "${COMPOSE}" up -d

sleep 5
findmnt "${MOUNT}"
sudo podman ps --filter name=topicfs_runtime
