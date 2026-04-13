#!/usr/bin/env bash
# deploy_typesupport.sh — deploy ROS2 package typesupport .so files for use by TopicFS
#
# Usage: ./deploy_typesupport.sh [PACKAGE_NAME] [INSTALL_DIR] [TYPESUPPORT_DIR]
#
# TopicFS discovers ROS2 services and topics at runtime and loads their typesupport
# .so files on demand. This script deploys the minimum required files from a package's
# colcon install tree into a subdirectory under TYPESUPPORT_DIR. The entrypoint of the
# TopicFS runtime container sources each subdirectory's setup.bash at startup, making
# all deployed packages available on AMENT_PREFIX_PATH.
#
# Example:
#   ./deploy_typesupport.sh swiftpro_resources \
#       /home/jack/dev/smithjack.net/SwiftRos2/install \
#       /opt/topicfs_typesupport
#
#   This deploys swiftpro_resources typesupport into:
#       /opt/topicfs_typesupport/swiftpro_resources/
#
# After deploying, restart the TopicFS runtime container to pick up the changes.
#
# Copyright 2025 Jack Sidman Smith — MIT License

set -euo pipefail

PACKAGE=${1:-swiftpro_resources}
INSTALL_DIR=${2:-/home/jack/dev/smithjack.net/SwiftRos2/install}
TYPESUPPORT_DIR=${3:-/opt/topicfs_typesupport}

SRC=$INSTALL_DIR/$PACKAGE
DST=$TYPESUPPORT_DIR/$PACKAGE

if [ ! -d "$SRC" ]; then
    echo "ERROR: package install directory not found: $SRC" >&2
    exit 1
fi

echo "Deploying typesupport: $PACKAGE"
echo "  from: $SRC"
echo "  to:   $DST"

# Clean previous deployment
sudo rm -rf "$DST"

# .so files — rosidl typesupport only
sudo mkdir -p "$DST/lib"
sudo cp "$SRC/lib"/lib${PACKAGE}__rosidl*.so "$DST/lib/"

# ament package index — required for package name lookup
sudo mkdir -p "$DST/share/ament_index/resource_index/packages"
sudo cp "$SRC/share/ament_index/resource_index/packages/$PACKAGE" \
    "$DST/share/ament_index/resource_index/packages/"

# package metadata
sudo mkdir -p "$DST/share/$PACKAGE"
sudo cp "$SRC/share/$PACKAGE/package.xml" "$DST/share/$PACKAGE/"

# minimal handwritten setup.bash — no colcon machinery needed
cat << 'EOF' | sudo tee "$DST/setup.bash" > /dev/null
#!/bin/bash
_p="$(builtin cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export AMENT_PREFIX_PATH="${_p}:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="${_p}/lib:${LD_LIBRARY_PATH:-}"
unset _p
EOF

echo "Done — restart topicfs_runtime to pick up changes"
