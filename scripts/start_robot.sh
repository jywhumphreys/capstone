#!/bin/bash
# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source /home/justin/ros2_ws/install/setup.bash

# Launch dashboard
exec ros2 launch robot_dashboard dashboard.launch.py
