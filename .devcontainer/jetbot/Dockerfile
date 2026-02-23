FROM dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=jetson_ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Add user to dialout group
RUN sudo usermod --append --groups dialout $USERNAME

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/install/setup.bash" >> ~/.bashrc && \
    sudo bash -c "echo 'source /opt/ros/${ROS_DISTRO}/install/setup.bash' >> /etc/bash.bashrc"

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################
RUN sudo pip install Jetson.GPIO
RUN sudo pip install pyserial

# ros2_control stack for hardware interface plugin and diff_drive_controller
RUN sudo apt-get update && sudo apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-xacro \
    && sudo rm -rf /var/lib/apt/lists/*
