#!/bin/bash

# ROS2 Humble container with mounted jetbot workspace
docker run --runtime nvidia -it --rm \
  --network=host \
  --privileged \
  --user root \
  -v /dev:/dev \
  -v /sys:/sys \
  -v /home/misbah/Documents/projects/misbah-jetbot/src/jetbot_ws:/ros_ws \
  -v /home/misbah/Documents/projects/misbah-jetbot/ros_container_configs:/ros_setup \
  --workdir /ros_ws \
  dustynv/ros:humble-ros-base-l4t-r32.7.1 \
  "$@"