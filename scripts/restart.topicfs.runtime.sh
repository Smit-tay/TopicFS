#!/usr/bin/env bash

sudo podman stop topicfs_runtime

sudo umount -l /home/jack/fuse_mount 2>/dev/null;

sudo umount -l /home/jack/fuse_mount 2>/dev/null;

sudo umount -l /home/jack/fuse_mount 2>/dev/null;

sudo umount -l /home/jack/fuse_mount 2>/dev/null;

sudo umount -l /home/jack/fuse_mount 2>/dev/null;

echo "--- mounts after cleanup ---" 
findmnt --all /home/jack/fuse_mount

sudo mount --bind /home/jack/fuse_mount /home/jack/fuse_mount
sudo mount --make-shared /home/jack/fuse_mount

echo "--- mounts after setup ---"
findmnt --all /home/jack/fuse_mount

sudo podman start topicfs_runtime

sleep 5

echo "--- mounts after container start ---"

findmnt --all /home/jack/fuse_mount
ls ~/fuse_mount
ls ~/fuse_mount/swiftpro
ls ~/fuse_mount/swiftpro/move_arm

