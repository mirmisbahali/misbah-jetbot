#!/usr/bin/env bash
set -e
# ROS 2 env
source /opt/ros/humble/setup.bash
# Common DDS settings (same on both machines)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}
# If later use Cyclone:
# export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
exec "$@"
