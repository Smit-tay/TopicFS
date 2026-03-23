# TopicFS
<img src="topic_fs/docs/images/repo_icon.png" alt="Repo Icon" width="30%">

TopicFS is a ROS2 FUSE filesystem interface. It mounts a directory where each 
active ROS2 topic appears as a subdirectory containing files:

- `latest` — the most recent message, base64-encoded JSON
- `info` — topic name and type
- `command` — present only for writable topics

This lets any POSIX-compatible tool — shell scripts, web servers, `cat`, `watch` 
— consume live ROS2 data without knowing anything about ROS2. That's the point.

## Why

ROS2 is powerful but hermetic. Getting data out of it typically means writing 
ROS2-aware code. TopicFS breaks that wall. If you can read a file, you can read 
a ROS2 topic.

## Architecture

TopicFS runs in a Podman container alongside your ROS2 nodes. The container 
mounts your project directory at the same path as the host, so there's no path 
remapping required.

Cross-machine ROS2 discovery uses standard mDNS via avahi. No discovery server, 
no hardcoded IPs.

## Prerequisites

- Ubuntu 24.04 (Noble) — the container base image
- ROS2 Jazzy Jalisco
- Podman and podman-compose
- `avahi-daemon` running on all machines that will run ROS2 nodes

### avahi requirement

ROS2's default middleware (Fast DDS) uses multicast for node discovery. avahi 
provides the mDNS stack that makes this work across machines on the same LAN. 
Without it, nodes on different machines cannot find each other — even on the 
same subnet.
```bash
# Fedora
sudo dnf install -y avahi
sudo systemctl enable --now avahi-daemon

# Ubuntu/Debian
sudo apt-get install -y avahi-daemon
sudo systemctl enable --now avahi-daemon
```

This is only required for cross-machine setups. Single-machine use works without it.

## Setup

### 1. Clone
```bash
git clone https://github.com/Smit-tay/TopicFS.git
cd TopicFS
```

### 2. Create environment file
```bash
echo -e "UID=$(id -u)\nGID=$(id -g)\nUSERNAME=$(whoami)" > .env
```

### 3. Build and start the container
```bash
podman-compose up -d --build
```

### 4. Build the package
```bash
podman exec topicfs bash -c "cd /home/$(whoami)/dev/smithjack.net/topicfs && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build"
```

## Usage

TopicFS mounts a FUSE filesystem at a configured mount point. Each active ROS2 
topic appears as a directory.

### Basic example

With a ROS2 talker running:
```bash
ros2 run demo_nodes_cpp talker
```

Run TopicFS:
```bash
source install/setup.bash
install/topic_fs/lib/topic_fs/topic_fs ~/fuse_mount
```

Browse topics:
```bash
ls ~/fuse_mount/
cat ~/fuse_mount/chatter/latest
cat ~/fuse_mount/chatter/info
```

### Example with SwiftRos2
 
SwiftRos2 is a ROS2 controller for uArm SwitfPro parallel linkage robotic arm

The primary use case is monitoring a live robotic arm. With SwiftRos2 running 
on a connected machine:
```bash
cat ~/fuse_mount/swiftpro/position/latest
watch cat ~/fuse_mount/joint_states/latest
```

See [SwiftRos2](https://github.com/Smit-tay/SwiftRos2) for the hardware node.

## Network Requirements

See the avahi section above. `ROS_DOMAIN_ID=11` is used across this project.

## Filesystem Structure
```
<mount_point>/
├── <topic_name>/
│   ├── latest       # most recent message, base64 JSON
│   ├── info         # topic name and type
│   └── command      # writable topics only
```

## Status

Active development. Core functionality works. Dynamic topic discovery is 
implemented — new topics appearing after startup are picked up automatically.

Known limitations:
- URDF parallel linkage cannot be accurately described (SwiftRos2 kinematics 
  node partially compensates)
- Unit tests not yet implemented

## IDE Setup

See [ide/README.md](ide/README.md) for Geany configuration with clangd LSP.
VSCode configuration is in [ide/vscode/](ide/vscode/).

## License

MIT. See [LICENSE](LICENSE).

## Author

Jack Sidman Smith — [smithjack.net](https://smithjack.net)
