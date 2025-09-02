# On the Jetson, this pulls the correct arm64 variant automatically
FROM dustynv/ros:humble-ros-base-l4t-r32.7.1

RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
