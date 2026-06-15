#!/usr/bin/env bash
set -e
cd ~/dev/smithjack.net/topicfs

echo "--- tag and push image to worker registry ---"
podman tag localhost/topicfs_runtime:latest worker:5000/topicfs_runtime:latest
podman push --tls-verify=false worker:5000/topicfs_runtime:latest

echo "--- rsync install tree to worker ---"
rsync -az --delete install/ jack@worker:/opt/topicfs/install/

echo "--- rsync typesupport to worker ---"
rsync -az --delete --rsync-path="sudo rsync" /opt/topicfs_typesupport/ jack@worker:/opt/topicfs_typesupport/

echo "--- rsync worker compose file ---"
rsync -az docker-compose.runtime.worker.yml jack@worker:/opt/topicfs/

echo "--- done ---"
echo
echo "On worker, run:"
echo "  /opt/topicfs/start.sh"
