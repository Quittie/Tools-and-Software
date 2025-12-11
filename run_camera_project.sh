#!/bin/bash
unset ROS_DOMAIN_ID
export ROS_DOMAIN_ID=0

source /opt/ros/humble/setup.bash
source ~/camera_project/install/setup.bash

ros2 launch camera_controller camera_turtle.launch.py
