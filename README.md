# TopicFS

<img src="topic_fs/docs/images/repo_icon.png" alt="Repo Icon" width="30%">

> *A ROS2 FUSE filesystem. Because apparently "just read a file" was too simple for the robotics world.*

TopicFS mounts a directory where every active ROS2 topic appears as a subdirectory. 
Inside each subdirectory: plain files. No ROS2 client libraries. No message type 
definitions. No colcon. No XML. Just files.

```
cat ~/fuse_mount/swiftpro/position/latest
```

That's it. That's the whole pitch.

---

## What You Get

Each topic directory contains:

| File | Contents |
|------|----------|
| `latest` | Most recent message, base64-encoded JSON |
| `info` | Topic name and type |
| `command` | Present only on writable topics |

New topics appearing after startup are picked up automatically. You don't have to 
restart anything.

---

## Why

ROS2 is a serious piece of engineering and also, if you've worked with it for more 
than a week, a serious source of friction. Getting data *out* of a running ROS2 
system typically means writing ROS2-aware code in a ROS2 workspace with ROS2 
dependencies and a ROS2 build system and—

TopicFS breaks that wall. Web servers, shell scripts, `cat`, `watch`, Python with 
no dependencies, a Bash one-liner on a machine that doesn't even have ROS2 installed: 
if it can read a file, it can read a ROS2 topic.

---

## Architecture

TopicFS runs in a Podman container (almost identical to Docker) alongside your ROS2 nodes. The container mounts 
your project directory at the same path as the host — no path remapping, no symlink 
gymnastics.

Cross-machine ROS2 discovery works via standard mDNS through `avahi-daemon`. No 
discovery server, no hardcoded IPs, no proprietary middleware configuration.

The FUSE filesystem implements the `poll(2)` operation, so applications using 
`poll()` or `select()` receive data-ready notifications directly when new messages 
arrive. `tail -f` won't work — it uses inotify, which the Linux kernel does not 
generate for FUSE filesystems. This is a kernel architecture constraint, not a 
TopicFS bug. Use `watch` instead:

```bash
watch -n 0.1 cat ~/fuse_mount/swiftpro/position/latest
```

---

## Prerequisites

- Ubuntu 24.04 (Noble) — the container base image
- ROS2 Jazzy Jalisco
- Podman and podman-compose
- `avahi-daemon` running on all machines participating in ROS2 discovery
- FUSE 3.x (`fuse3` package)

### avahi

ROS2's default middleware (Fast DDS) uses multicast for node discovery. avahi 
provides the mDNS stack that makes cross-machine discovery work. Without it, nodes 
on different machines cannot find each other — even on the same subnet.

```bash
# Fedora
sudo dnf install -y avahi
sudo systemctl enable --now avahi-daemon

# Ubuntu/Debian
sudo apt-get install -y avahi-daemon
sudo systemctl enable --now avahi-daemon
```

Single-machine setups don't need this.

---

## Setup

### 1. Clone

```bash
git clone https://github.com/Smit-tay/TopicFS.git
cd TopicFS
```

### 2. Create the environment file

```bash
echo -e "UID=$(id -u)\nGID=$(id -g)\nUSERNAME=$(whoami)" > .env
```

### 3. Build and start the container

```bash
podman-compose up -d --build
```

### 4. Build the package inside the container

```bash
podman exec topicfs bash -c "
    cd /home/$(whoami)/dev/smithjack.net/topicfs && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build
"
```

---

## Running

### Basic smoke test

Terminal 1 — run a talker:
```bash
ros2 run demo_nodes_cpp talker
```

Terminal 2 — run TopicFS inside the container:
```bash
source install/setup.bash
ros2 run topic_fs topic_fs
```

Terminal 3 — read it like a file, because it is one:
```bash
cat ~/fuse_mount/chatter/latest
cat ~/fuse_mount/chatter/info
watch -n 0.1 cat ~/fuse_mount/chatter/latest
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `mount_point:=` | `~/fuse_mount` | Where to mount the filesystem |
| `fuse_threads:=` | `max(2, nproc/2)` | FUSE worker threads. Minimum 2 required for poll support |
| `notification_interval_ms:=` | `100` | Minimum ms between cache invalidation events per topic |
| `discovery_interval_ms:=` | `1000` | How often to scan for new topics |
| `writable_topics:=` | `[]` | Topics to expose as writable via `command` file |

```bash
ros2 run topic_fs topic_fs \
    mount_point:=/tmp/ros_topics \
    fuse_threads:=4 \
    notification_interval_ms:=50
```

### With a live robotics stack

The reason TopicFS exists.

If you have a ROS2 hardware node running — a robotic 
arm, a sensor array, a mobile base, anything publishing topics — TopicFS exposes 
all of it as a filesystem the moment it connects. No code changes to the hardware 
node. No new dependencies. It just appears.

[SwiftRos2](https://github.com/Smit-tay/SwiftRos2) is the reference integration: 
a ROS2 hardware abstraction layer for the UFactory UArm Swift Pro, publishing arm 
position, joint states, pump status, and connection state as topics. With it 
running on any reachable machine (`ROS_DOMAIN_ID=11`):

```bash
# Current arm position
cat ~/fuse_mount/swiftpro/position/latest

# Joint states, updating continuously
watch -n 0.1 cat ~/fuse_mount/joint_states/latest

# Topic metadata
cat ~/fuse_mount/swiftpro/connected/info
```

---

## Filesystem Structure

```
<mount_point>/
├── chatter/
│   ├── latest       ← most recent message, base64 JSON
│   └── info         ← topic name and type
├── swiftpro/
│   ├── position/
│   │   ├── latest
│   │   └── info
│   ├── connected/
│   │   ├── latest
│   │   └── info
│   └── pump_status/
│       ├── latest
│       └── info
└── some_writable_topic/
    ├── latest
    ├── info
    └── command       ← writable topics only
```

---

## Network Setup

`ROS_DOMAIN_ID=11` is used across this project. Set it consistently on all nodes:

```bash
export ROS_DOMAIN_ID=11
```

The TopicFS container sets this automatically. The SwiftRos2 container requires 
it explicitly:

```bash
podman run -e ROS_DOMAIN_ID=11 ...
```

Both machines must have `avahi-daemon` running for cross-machine discovery. If 
topics from remote nodes aren't appearing, that's the first thing to check.

---

## Known Limitations

- `tail -f` does not work on FUSE filesystems. Use `watch -n 0.1 cat` instead.
  This is a kernel inotify/FUSE architecture issue and is not fixable in userspace.
- Unit tests not yet implemented.
- The `command` file interface assumes atomic writes. Don't pipe large messages 
  in chunks and expect it to work.

---

## IDE Setup

See [ide/README.md](ide/README.md) for Geany + clangd LSP configuration.  
VSCode configuration is in [ide/vscode/](ide/vscode/).

---

## Status

Active development. Core functionality works:

- FUSE filesystem mount/unmount with clean signal handling
- Dynamic topic discovery — new topics picked up without restart
- Generic subscription to any topic type
- `poll(2)` / `select(2)` support for data-ready notification
- Writable topic support via `command` file
- Cross-machine ROS2 discovery via avahi/mDNS
- Configurable FUSE thread count with hardware-aware defaults

---

## License

MIT. See [LICENSE](LICENSE).

## Author

Jack Sidman Smith — [smithjack.net](https://smithjack.net)
