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
RUN echo "source /opt/ros/${ROS_DISTRO}/install/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################
RUN sudo pip install Jetson.GPIO
RUN sudo pip install pyserial

# Build ros2_control stack from source (apt binaries conflict with dustynv L4T base)
RUN sudo apt-get update && sudo apt-get install -y \
    python3-colcon-common-extensions \
    python3-vcstool \
    && sudo rm -rf /var/lib/apt/lists/*

RUN mkdir -p ~/ros2_overlay_ws/src && \
    cd ~/ros2_overlay_ws/src && \
    git clone -b jazzy --depth 1 https://github.com/ros-controls/ros2_control.git && \
    git clone -b jazzy --depth 1 https://github.com/ros-controls/ros2_controllers.git && \
    git clone -b jazzy --depth 1 https://github.com/ros/robot_state_publisher.git && \
    git clone -b jazzy --depth 1 https://github.com/ros/xacro.git

RUN bash -c "\
    source /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    cd ~/ros2_overlay_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    "

# Source the overlay after the base ROS setup
RUN echo "source ~/ros2_overlay_ws/install/setup.bash" >> ~/.bashrc


