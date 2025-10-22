# Start with an official ROS 2 base image for Jazzy Jalisco
FROM ros:jazzy-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    ROS_DISTRO=jazzy

# Define build arguments for host UID, GID, and username with defaults
ARG HOST_UID=1000
ARG HOST_GID=1000
ARG USERNAME=user

# Install essential packages and ROS development tools
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    bash-completion \
    curl \
    fuse3 \
    gdb \
    git \
    libfuse3-dev \
    nlohmann-json3-dev \
    openssh-client \
    python3-colcon-argcomplete \
    python3-colcon-common-extensions \
    sudo \
    vim \
    && apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /topicfs

# Setup user configuration
# Create or rename group and user, handle existing UID/GID conflicts
RUN echo "Configuring group with GID=$HOST_GID for $USERNAME" && \
    (getent group $HOST_GID && groupmod -n $USERNAME $(getent group $HOST_GID | cut -d: -f1) || groupadd --gid $HOST_GID $USERNAME) && \
    echo "Configuring user $USERNAME with UID=$HOST_UID, GID=$HOST_GID" && \
    (id -u $HOST_UID >/dev/null 2>&1 && \
     echo "UID $HOST_UID exists, updating user" && \
     usermod -l $USERNAME -d /home/$USERNAME -m -g $HOST_GID $(id -un $HOST_UID) || \
     useradd --uid $HOST_UID --gid $HOST_GID -m -s /bin/bash $USERNAME) && \
    echo "Configuring sudo and bashrc for $USERNAME" && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc && \
    echo "Setting ownership of /topicfs to $USERNAME:$HOST_GID" && \
    chown -R $USERNAME:$HOST_GID /topicfs && \
    echo "User setup complete: $(id $USERNAME)"


USER $USERNAME

# Install ROS 2 dependencies
RUN sudo apt-get update && \
    sudo apt-get install -y --no-install-recommends \
    ros-jazzy-ament-cmake-clang-format \
    ros-jazzy-ament-cmake-clang-tidy \
    ros-jazzy-controller-interface \
    ros-jazzy-controller-manager \
    ros-jazzy-control-msgs \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-generate-parameter-library \
    ros-jazzy-hardware-interface \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-joint-trajectory-controller \
    ros-jazzy-moveit-kinematics \
    ros-jazzy-moveit-planners-ompl \
    ros-jazzy-moveit-ros-move-group \
    ros-jazzy-moveit-ros-visualization \
    ros-jazzy-moveit-simple-controller-manager \
    ros-jazzy-realtime-tools \
    ros-jazzy-ros-gz \
    ros-jazzy-ros2-control-test-assets \
    ros-jazzy-ros2controlcli \
    ros-jazzy-rviz2 \
    ros-jazzy-sdformat-urdf \
    ros-jazzy-xacro \
    && sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

# Install missing ROS 2 dependencies
COPY . /topicfs/src
RUN sudo chown -R $USERNAME:$USERNAME /topicfs && \
    sudo apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/* && \
    rm -rf /home/$USERNAME/.ros 

# Set the default shell to bash and the workdir to the source directory
SHELL [ "/bin/bash", "-c" ]
ENTRYPOINT []
WORKDIR /topicfs