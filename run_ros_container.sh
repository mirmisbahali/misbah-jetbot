#!/bin/bash

# ROS2 Humble container with mounted jetbot workspace
docker run --runtime nvidia -it --rm \
  --network=host \
  --privileged \
  --user root \
  -v /dev:/dev \
  -v /sys:/sys \
  -v ~/projects/misbah-jetbot/src/jetbot_ws:/ros_ws \
  -v ~/projects/misbah-jetbot/ros_container_configs:/ros_setup \
  -v /dev:/dev -v /sys:/sys \
  --workdir /ros_ws \
  dustynv/ros:humble-ros-base-l4t-r32.7.1 \
  "$@"