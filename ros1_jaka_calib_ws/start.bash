#!/usr/bin/env bash
# This script is run at the start of the container
source /opt/ros/noetic/setup.bash && \
source ./devel/setup.bash && \
roslaunch zhw_jaka_ops start.launch use_rviz:=false video_device:=$VIDEO_DEVICE
