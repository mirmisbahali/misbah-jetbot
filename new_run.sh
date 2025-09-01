docker run --runtime nvidia -it --rm \
  --network=host \
  --privileged \
  --user "$(id -u):$(id -g)" \
  -e USER="$(id -un)" -e HOME=/ros_ws -e TERM=xterm-256color \
  -v /etc/passwd:/etc/passwd:ro \
  -v /etc/group:/etc/group:ro \
  -v /dev:/dev -v /sys:/sys \
  -v ~/projects/misbah-jetbot/src/jetbot_ws:/ros_ws \
  -v ~/projects/misbah-jetbot/ros_container_configs:/ros_setup \
  --workdir /ros_ws \
  dustynv/ros:humble-ros-base-l4t-r32.7.1 \
  /bin/bash

