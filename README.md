# TopicFS
<img src="topic_fs/docs/images/repo_icon.png" alt="Repo Icon" width="30%">

TopicFS is a ROS2 package that provides a FUSE (Filesystem in Userspace) interface to access ROS2 topics as a virtual filesystem. Each topic is represented as a directory, containing two files: `latest` (the most recent message in JSON format) and `info` (metadata about the topic). This allows developers to interact with ROS2 data using standard filesystem tools (e.g., `ls`, `cat`), simplifying debugging and integration with non-ROS tools.

## Purpose
The primary goal of TopicFS is to offer a novel way to access ROS2 topic data, making it easier to:
- Inspect topic messages without writing ROS2-specific code.
- Integrate ROS2 data with scripts or tools that expect file-based interfaces.
- Debug ROS2 applications by browsing topics as files.

Currently, TopicFS supports any topic type, although is still under development, so may be unstable.

## Prerequisites
- **Operating System**: Ubuntu 24.04 (Noble)
- **ROS2 Distribution**: Jazzy Jalisco
- **Docker**: Required for the provided containerized setup
- **Dependencies**:
  - `fuse3`
  - `libfuse-dev` (minimum FUSE 2.9.9)
  - `nlohmann-json3-dev`
  - `ros-jazzy-demo-nodes-cpp` (for the chatter example)

## Installation
TopicFS is designed to run in a Docker container to ensure a consistent environment.<br>
If run outside a container, you may need to install the following packages:
  - fuse
  - libfuse-dev
  - nlohmann-json3-dev

Follow these steps to set up the project.

### 1. Clone the Repository
```bash
git clone <repository-url> topicfs
cd topicfs
```

### 2. Set Up the Docker Environment
The project includes a `Dockerfile` and `docker-compose.yml` to create a ROS2 Jazzy container.

1. **Create an `.env` File**:
   ```bash
   echo -e "UID=$(id -u)\nGID=$(id -g)\nUSERNAME=$(whoami)" > .env
   ```

2. **Build and Start the Container**:
   ```bash
   docker-compose up -d --build
   ```

3. **Enter the Container**:
   ```bash
   docker exec -it topicfs /bin/bash
   ```

### 3. Build the Package
Inside the container:
```bash
cd /topicfs
colcon build
source install/setup.bash
```

## Usage
TopicFS mounts a FUSE filesystem at a specified mount point (e.g., `/mnt/topicfs`), where ROS2 topics appear as directories.

### Running the Chatter Example
This example uses the `demo_nodes_cpp` package to publish a `std_msgs/String` topic (`/chatter`) and mounts the FUSE filesystem to access it.

1. **Run the Chatter Publisher**:
   In one terminal (inside the container):
   ```bash
   source /opt/ros/jazzy/setup.bash
   ros2 run demo_nodes_cpp talker
   ```

2. **Run TopicFS**:
   In another terminal (inside the container):
   ```bash
   cd /topicfs
   source install/setup.bash
   install/topic_fs/lib/topic_fs/topic_fs /mnt/topicfs
   ```

3. **Browse the Filesystem**:
   In a third terminal:
   ```bash
   ls /mnt/topicfs
   ```
   - Should show: `chatter`
   ```bash
   ls /mnt/topicfs/chatter
   ```
   - Should show: `info latest`
   ```bash
   cat /mnt/topicfs/chatter/latest
   ```
   - Should show: `{"data": "Hello, world! ..."}`
   ```bash
   cat /mnt/topicfs/chatter/info
   ```
   - Should show: `Topic: chatter\nType: std_msgs/String\n`

4. **Shut Down**:
   - Press `Ctrl+C` in the `topic_fs` terminal.
   - Unmount the filesystem:
     ```bash
     sudo fusermount -u /mnt/topicfs
     ```
   - Stop the publisher:
     ```bash
     pkill -f talker
     ```

### Debugging
To run with FUSE debug output:
```bash
install/topic_fs/lib/topic_fs/topic_fs /mnt/topicfs -d
```

## Filesystem Structure
The FUSE filesystem is structured as follows:
- `/`: Root directory containing topic directories.
- `/<topic>`: Directory for each `std_msgs/String` topic (e.g., `/chatter`).
  - `latest`: File with the latest message in JSON format.
  - `info`: File with metadata (`Topic: <topic>\nType: std_msgs/String\n`).

Example:
```
/mnt/topicfs/
├── chatter
│   ├── latest
│   ├── info
```

## Quality Assurance
- **Code Style**: Follows (modified) Stroustrup C++ style guidelines,<br>
                  enforced with `ament_cmake_clang_format`.  (see `uncrustify.cfg`)
- **Linting**: Uses `ament_lint_auto` and `ament_lint_common` for static analysis.
- **Testing**: Planned for future releases (e.g., `ament_cmake_gtest` for unit tests).
- **Build System**: Uses `ament_cmake` for ROS2 integration.

## Contributing
Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a feature branch (`git checkout -b feature/YourFeature`).
3. Commit changes (`git commit -m "Add YourFeature"`).
4. Push to the branch (`git push origin feature/YourFeature`).
5. Open a pull request.

See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed guidelines.

## Code of Conduct
This project adheres to the [ROS2 Code of Conduct](https://www.ros.org/conduct.html). Please ensure all interactions are respectful and inclusive.

## License
TopicFS is licensed under the MIT license See [LICENSE](LICENSE) for details.

## Contact
For questions or feedback, please open an issue on the repository or contact the maintainer via GitHub:

## Acknowledgments
- Inspired by the need for simple ROS2 data access.
- Built with [libfuse](https://github.com/libfuse/libfuse) and [nlohmann/json](https://github.com/nlohmann/json).
- Thanks to the ROS2 community for tools and support.

## Topic FS Utility
Copyright (c) 2025 Jack Sidman Smith<br>
Licensed under the MIT License.
