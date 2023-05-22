#!/usr/bin/env bash

# Setup ROS for Local Development
source /opt/ros/noetic/setup.bash
source ~/minecROS_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:5802
export ROSLAUNCH_SSH_UNKNOWN=1

exec "$@"
