# TopicFS

**A POSIX Filesystem Interface for ROS2 — by Jack Sidman Smith**

#### TopicFS exposes ROS2 topics, services, and actions as POSIX files.

> *"What if you could just `cat` a robot?"*

TopicFS exposes live ROS2 topics, services, and actions as ordinary files and
directories on your filesystem. Once it is running, any user on the host machine
can read sensor data, publish messages, and call robot services using nothing more
than `cat`, `echo`, and standard shell scripts — with absolutely no ROS2 installed
on the host. Not a single ROS2 package. Not a single Python dependency. Nothing.

Just files.

```bash
# Read live joint angles from a robot arm
cat ~/fuse_mount/joint_states/latest
{"name":["Joint1","Joint2","Joint3"],"position":[1.45,1.12,1.26],...}

# Get the arm's current XYZ position
cat ~/fuse_mount/swiftpro/position/latest
{"x":178.8,"y":-22.0,"z":8.8}

# Move the arm — by writing JSON to a file
echo '{"x":200.0,"y":0.0,"z":150.0,"speed":1000,"wait":true}' \
  > ~/fuse_mount/swiftpro/move_to/command

# Check if it worked
cat ~/fuse_mount/swiftpro/move_to/response
{"error_code":0,"message":"OK","success":true}

# Turn the vacuum pump on
echo '{"on": true}' > ~/fuse_mount/swiftpro/set_pump/command
```

That is it. No `ros2 topic echo`. No `rclpy`. No environment sourcing. No DDS
configuration. No ROS2 anything. The robot is just a filesystem.

This is not a hack or a workaround. It is a deliberate architectural choice that
makes ROS2 robotics data accessible to any tool, any language, any user — while
keeping all the ROS2 complexity safely isolated inside a container where it belongs.

---

## Author's Note

This project was designed from scratch by Jack Sidman Smith as part of a broader
robotics stack integrating a FUSE filesystem, a complete ROS2 robot arm driver,
and a physical time-display demo (SteampunkClock). The insight that ROS2's
runtime typesupport loading mechanism (`dlopen` + `AMENT_PREFIX_PATH`) could be
exploited to build a fully generic, zero-compile-time-dependency ROS2 filesystem
interface is — let's be honest — pretty clever. The result is a system where a
completely ROS2-naive user can interact with a live robot by reading and writing
files, which is both technically elegant and genuinely useful.

---

## How It Works

TopicFS is a [FUSE](https://github.com/libfuse/libfuse) filesystem implemented
as a ROS2 node. It runs inside a container, connects to the ROS2 DDS network,
and discovers all available topics and services automatically. It then mounts a
virtual filesystem that maps each topic and service to a directory of files.

The magic is in two places:

**1. Runtime typesupport loading**

ROS2 messages are serialized in CDR (Common Data Representation) binary format
on the wire. To convert CDR to human-readable JSON, TopicFS needs the message
type's compiled `.so` typesupport library. Rather than linking against any
specific message packages at compile time (which would make TopicFS specific to
one robot), TopicFS uses `dlopen()` at runtime to load whatever `.so` files it
needs, keyed by the type string it discovers from DDS (`"geometry_msgs/msg/Point"`,
`"sensor_msgs/msg/JointState"`, etc.).

If the `.so` is available, the message is decoded to readable JSON. If not, the
raw CDR bytes are stored as base64 with a clear label — and the next time a
message arrives, TopicFS tries again. No restart required when you add new
typesupport.

**2. FUSE mount propagation**

The FUSE filesystem is mounted inside a container. Through careful use of Linux
shared mount propagation and rootful Podman, the mount point is visible on the
host filesystem as an ordinary directory. The host user never interacts with the
container. They just read files.

---

## Filesystem Layout

```
~/fuse_mount/
├── joint_states/
│   ├── latest          ← latest message as JSON (read-only, live)
│   └── info            ← topic type string
├── swiftpro/
│   ├── position/
│   │   ├── latest      ← {"x":..., "y":..., "z":...}
│   │   └── info
│   ├── connected/
│   │   └── latest      ← {"data": true}
│   ├── pump_status/
│   │   └── latest      ← {"data": false}
│   ├── move_to/        ← ROS2 service
│   │   ├── command     ← write JSON request here
│   │   └── response    ← read JSON response from here
│   ├── set_pump/
│   │   ├── command
│   │   └── response
│   ├── set_gripper/
│   │   ├── command
│   │   └── response
│   └── reset/
│       ├── command
│       └── response
└── ...                 ← any other topics/services discovered on the DDS network
```

Topics with publishers registered in the `writable_topics` parameter also get a
`command` file for publishing messages.

---

## Container Architecture

TopicFS uses two completely separate container images. This is not over-engineering
— it is the correct approach and the size numbers make the argument on their own.

### Why Two Images?

| | Development | Runtime |
|---|---|---|
| **Size** | ~1.6 GB | ~518 MB |
| **Base** | `ros:jazzy-ros-base` | `ros:jazzy-ros-core` |
| **Contains** | Full toolchain, clangd, colcon, nlohmann, libfuse3-dev | fuse3, ROS2 runtime libs only |
| **Source code** | Mounted from host | Not present |
| **Binary** | Built by colcon, stored on host | Mounted from host install tree |
| **Rebuilt when** | Dockerfile changes | Dockerfile.runtime changes |
| **Runs as** | Rootless podman | **Rootful podman** (required — see below) |

Neither image bakes the TopicFS binary in. The dev container builds it with
`colcon build` and the output lands on the host filesystem (the source tree is
mounted). The runtime container mounts that same install tree and runs the binary
directly. A code change requires only a colcon rebuild and a container restart —
never an image rebuild.

### Development Image (`Dockerfile`)

Used on the development machine for building TopicFS. Contains the full ROS2
Jazzy development toolchain, clangd for IDE integration (Geany + LSP), colcon,
and all build-time dependencies including `libfuse3-dev` and `nlohmann-json3-dev`.

```bash
# Build the dev image
podman build --format docker \
  --build-arg HOST_UID=$(id -u) \
  --build-arg HOST_GID=$(id -g) \
  --build-arg USERNAME=$(id -un) \
  -t topicfs_topicfs .

# Start the dev container
podman-compose up -d

# Build TopicFS inside the dev container
podman exec topicfs bash -c \
  "cd /home/jack/dev/smithjack.net/topicfs && \
   source /opt/ros/jazzy/setup.bash && \
   colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
```

The entire project directory is mounted into the container. Build output lands
in `topicfs/build/` and `topicfs/install/` on the **host** filesystem and
survives container restarts.

### Runtime Image (`Dockerfile.runtime`)

Used in production. Contains only what is needed to run the `topic_fs` binary:
`fuse3` and the minimal ROS2 runtime libraries. No build tools, no IDE support,
no headers.

```bash
# Build the runtime image
podman build --format docker \
  -f Dockerfile.runtime \
  -t topicfs_runtime:latest .
```

---

## The FUSE Mount — Host Visibility and Permissions

This is the most important operational detail. Read it carefully.

### Why Rootful Podman?

TopicFS creates a FUSE filesystem inside the container. For that mount to be
visible on the host machine, Linux mount namespace propagation must work in the
outward direction (container → host). **This does not work with rootless Podman.**

In rootless Podman, the container runs in a user namespace where the container
user maps to a subordinate UID range on the host. Mount operations inside this
namespace are not visible outside it. This is a fundamental Linux kernel
limitation, not a Podman bug.

The solution is to run the runtime container with `sudo podman` (rootful Podman).
The container still runs as root inside, but the FUSE mount propagates correctly
to the host. This is the correct architectural choice for a production service
container — it is not a workaround.

### Shared Mount Propagation

The host directory used as the FUSE mount point must be configured as a **shared
bind mount** before the runtime container starts. Without this, the FUSE mount
created inside the container will not propagate back to the host.

```bash
# Run once before starting the runtime container (does not survive reboot — see below)
sudo mount --bind /home/jack/fuse_mount /home/jack/fuse_mount
sudo mount --make-shared /home/jack/fuse_mount

# Verify
findmnt /home/jack/fuse_mount
# Should show the mount with shared propagation
```

**Making this persistent across reboots** — add a systemd mount unit or
`/etc/fstab` entry. The quadlet approach is recommended (same as the registry
service):

```ini
# /etc/systemd/system/home-jack-fuse_mount.mount
[Unit]
Description=Shared bind mount for TopicFS FUSE propagation
Before=topicfs-runtime.service

[Mount]
What=/home/jack/fuse_mount
Where=/home/jack/fuse_mount
Type=none
Options=bind,shared

[Install]
WantedBy=multi-user.target
```

### `allow_other` — Non-Root Host Access

By default, a FUSE filesystem mounted by root is only accessible to root. To
allow normal users on the host to read and write the TopicFS mount, two things
are required:

**1. Enable `user_allow_other` in `/etc/fuse.conf`:**

```bash
sudo sed -i 's/#user_allow_other/user_allow_other/' /etc/fuse.conf
```

**2. TopicFS must be launched with the `allow_other` FUSE option** — this is
set in `topicfs.cpp` `initialize_fuse()`:

```cpp
std::vector<char*> fuse_argv = {
  (char*)"topic_fs",
  (char*)"-o",
  (char*)"entry_timeout=0,attr_timeout=0,allow_other",
  nullptr
};
```

With both of these in place, any user on the host can `cat`, `echo`, and `ls`
the TopicFS mount point without any special permissions.

### Starting the Runtime Container

```bash
# Ensure mount point exists and is shared (see above)
mkdir -p /home/jack/fuse_mount

sudo podman run -d \
  --name topicfs_runtime \
  --network host \
  --privileged \
  --device /dev/fuse \
  -v /home/jack/fuse_mount:/home/jack/fuse_mount:shared \
  -v /home/jack/dev/smithjack.net/topicfs/install:/home/jack/dev/smithjack.net/topicfs/install \
  -v /opt/topicfs_typesupport:/opt/topicfs_typesupport \
  -e ROS_DOMAIN_ID=11 \
  -e TOPICFS_MOUNT_POINT=/home/jack/fuse_mount \
  --cap-add SYS_ADMIN \
  localhost/topicfs_runtime:latest
```

Key flags:
- `--privileged` + `--device /dev/fuse` + `--cap-add SYS_ADMIN` — required for FUSE
- `-v .../fuse_mount:.../:shared` — enables mount propagation to host
- `--network host` — DDS discovery requires host network access
- `ROS_DOMAIN_ID=11` — must match all other ROS2 nodes on the network

---

## Adding Support for Third-Party ROS2 Nodes

This is the killer feature. TopicFS can talk to **any** ROS2 node on the network
without being recompiled. You just drop the typesupport `.so` files in the right
place and restart.

### How It Works

When TopicFS discovers a topic with type `"swiftpro_resources/srv/MoveTo"`, it
calls `dlopen("libswiftpro_resources__rosidl_typesupport_cpp.so", RTLD_LAZY)`.
The dynamic linker finds this `.so` automatically if the package's install prefix
is on `LD_LIBRARY_PATH` — which `AMENT_PREFIX_PATH` / `setup.bash` sets up.

If the `.so` is not found, the message is stored as base64-encoded CDR with a
clear label showing which package is missing:

```json
{"_encoding": "base64_cdr", "_type": "swiftpro_resources/srv/MoveTo", "data": "..."}
```

This tells you exactly what to add.

### Step-by-Step: Adding SwiftRos2 Support

**Step 1 — Build the package typesupport in the SwiftRos2 dev container:**

```bash
# The full colcon build already produces this — check first:
ls /home/jack/dev/smithjack.net/SwiftRos2/install/setup.bash
```

**Step 2 — Copy the install tree to the typesupport directory:**

```bash
cp -r /home/jack/dev/smithjack.net/SwiftRos2/install \
      /opt/topicfs_typesupport/swiftpro_resources
```

The critical files are the `.so` libraries:
```
/opt/topicfs_typesupport/swiftpro_resources/swiftpro_resources/lib/
    libswiftpro_resources__rosidl_typesupport_cpp.so
    libswiftpro_resources__rosidl_typesupport_introspection_cpp.so
    ...
```

**Step 3 — Restart the runtime container:**

The `entrypoint.sh` automatically sources any `setup.bash` found under
`/opt/topicfs_typesupport/*/setup.bash` at startup, adding the `.so` directories
to `LD_LIBRARY_PATH` and `AMENT_PREFIX_PATH`.

```bash
sudo podman stop topicfs_runtime
sudo podman rm topicfs_runtime
# ... run command as above ...
```

**Step 4 — Verify:**

```bash
sudo podman logs topicfs_runtime | grep "new service"
# Should show all SwiftRos2 services discovered without errors
```

### For Any Other ROS2 Package

The process is identical:

1. Build the package with `colcon build` on the same architecture (x86_64)
   and same ROS2 distro (Jazzy) — the `.so` ABI is architecture and distro specific
2. Copy the colcon install tree to `/opt/topicfs_typesupport/<package_name>/`
3. Restart the runtime container

The `.so` files must be built for the **same architecture and ROS2 distro** as
the machine running TopicFS. ARM builds (Jetson, Raspberry Pi) will not work on
x86_64, and Humble builds will not work in a Jazzy runtime.

---

## Development Workflow

### Normal Cycle (after initial setup)

```
Edit TopicFS source on host (Geany)
    → NF_00  colcon build inside dev container
    → Restart runtime container
    → cat ~/fuse_mount/... to verify
```

### Geany Build Menu Reference

| Entry | Action |
|---|---|
| NF_00 TopicFS - Build | colcon build in dev container |
| NF_01 TopicFS - Clean | remove build/ install/ log/ |
| NF_02 TopicFS - Lint | colcon test |
| NF_03 TopicFS - Run | restart runtime container |

### The `entrypoint.sh`

The runtime container startup script:

```bash
#!/bin/bash
source /opt/ros/jazzy/setup.bash

# Fail clearly if TopicFS has not been built yet
if [ ! -f /path/to/topicfs/install/setup.bash ]; then
    echo "ERROR: TopicFS install not found. Run NF_00 (colcon build) first."
    exit 1
fi

source /path/to/topicfs/install/setup.bash

# Auto-source any third-party typesupport packages
for setup in /opt/topicfs_typesupport/*/setup.bash; do
    [ -f "$setup" ] && source "$setup"
done

exec ros2 run topic_fs topic_fs \
    --ros-args -p mount_point:=${TOPICFS_MOUNT_POINT:-/home/jack/fuse_mount}
```

---

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `mount_point` | `~/fuse_mount` | FUSE filesystem mount point |
| `writable_topics` | `[]` | Topics to publish to (creates `command` file) |
| `notification_interval_ms` | `100` | Minimum ms between inode invalidation notifications |
| `discovery_interval_ms` | `1000` | How often to scan for new topics/services |
| `fuse_threads` | `hw_concurrency/2` | FUSE worker threads (minimum 2 for poll support) |

---

## Integration with SwiftRos2

TopicFS and SwiftRos2 run in separate containers and communicate via DDS over
`network_mode: host`. No special configuration is required — TopicFS discovers
SwiftRos2 topics and services automatically via standard ROS2 graph discovery.

Both containers must use the same `ROS_DOMAIN_ID` (default: 11).

The complete integration chain:

```
UArm Swift Pro (USB serial)
    ↓
swiftpro_hardware node (worker container)
    publishes → /joint_states, /swiftpro/position, etc.
    serves    → /swiftpro/move_to, /swiftpro/set_pump, etc.
    ↓ DDS (network_mode: host, ROS_DOMAIN_ID=11)
TopicFS runtime container (NUC, rootful podman)
    discovers topics and services automatically
    mounts FUSE filesystem
    ↓ shared mount propagation
~/fuse_mount/ (NUC host filesystem)
    ↓
Any user, any tool, any language
    cat ~/fuse_mount/joint_states/latest
    echo '{"on":true}' > ~/fuse_mount/swiftpro/set_pump/command
```

---

## Actions

ROS2 actions are currently exposed as their raw `/_action/` sub-services and
sub-topics. They appear in the filesystem and are interactable, but the UX
requires knowledge of the action protocol (goal UUIDs, status polling, etc.).

Full ergonomic action support — with a clean `send_goal` / `feedback` / `result`
directory structure — is planned as a future task. The `discover_actions()` stub
is already in place.

---

## Known Issues and Operational Notes

**FUSE mount not visible on host after restart**
The shared bind mount does not survive reboots. Run the following before starting
the runtime container, or make it persistent via systemd:
```bash
sudo mount --bind /home/jack/fuse_mount /home/jack/fuse_mount
sudo mount --make-shared /home/jack/fuse_mount
```

**"Transport endpoint is not connected" on fuse_mount**
The FUSE filesystem was unmounted (container stopped) but the mount point is in
a stale state. Fix:
```bash
sudo fusermount3 -u /home/jack/fuse_mount 2>/dev/null
sudo umount -l /home/jack/fuse_mount 2>/dev/null
```

**Podman SHELL warning during image build**
`podman-compose` 1.5.0 does not pass `--format docker` to `podman build`. The
warning is cosmetic — images build and run correctly. Use `podman build --format
docker` directly for image builds.

**Services show as base64 instead of JSON**
The typesupport `.so` for that package is not in `/opt/topicfs_typesupport`.
The `_type` field in the base64 JSON tells you exactly which package to add.

**TopicFS runtime must use `sudo podman`**
Rootless Podman cannot propagate FUSE mounts to the host. This is a Linux kernel
user namespace limitation. Always use `sudo podman` for the runtime container.

---

## Directory Structure

```
topicfs/
├── Dockerfile                    # Development image
├── Dockerfile.runtime            # Lean runtime image
├── docker-compose.yml            # Dev container (rootless podman)
├── docker-compose.runtime.yml    # Runtime container config (reference)
├── entrypoint.sh                 # Runtime startup script
├── scripts/
│   └── setup_typesupport.sh      # Helper for sourcing typesupport packages
├── README.md                     # This file
└── topic_fs/
    ├── CMakeLists.txt
    ├── package.xml
    ├── include/topic_fs/
    │   ├── ros_message_converter.hpp   # Runtime CDR↔JSON conversion
    │   ├── topicfs.hpp                 # TopicFS class (FUSE lifecycle)
    │   ├── topicfs_fuse.hpp            # FUSE callback declarations
    │   └── topicfs_node.hpp            # ROS2 node (discovery, subscriptions)
    └── src/
        ├── main.cpp
        ├── ros_message_converter.cpp   # dlopen + introspection-based conversion
        ├── topicfs.cpp                 # FUSE init, signal handling, threading
        ├── topicfs_fuse.cpp            # FUSE callbacks: getattr, read, write, poll
        └── topicfs_node.cpp            # Topic/service discovery and management
```

---

## Why This Is Significant

The standard way to interact with a ROS2 robot requires:
- ROS2 installed (hundreds of packages, gigabytes)
- The correct ROS distro sourced in your environment
- Knowledge of `ros2 topic echo`, `ros2 service call`, message type syntax
- Often: Python, rclpy, or a compiled C++ node

TopicFS reduces this to:
- `cat`
- `echo`
- Nothing else

This has real implications:

- **Shell scripts** can read sensor data and trigger robot actions
- **Web servers** can serve live robot data as static files
- **Any programming language** that can read a file can consume ROS2 data
- **Non-robotics users** can interact with a robot without any training
- **Monitoring tools** (Nagios, Grafana, anything file-based) work out of the box
- **`tail -f`** gives you a live stream of any topic

The key technical insight is that ROS2's own `AMENT_PREFIX_PATH` mechanism,
combined with `dlopen`, provides a clean path to fully generic message
handling at runtime. TopicFS exploits this to remain completely agnostic about
what robot it is talking to — the same binary works with any ROS2 system, any
message type, any robot, forever.

