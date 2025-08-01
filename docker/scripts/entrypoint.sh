#!/bin/bash
set -e

# Source ROS
source /opt/ros/humble/setup.bash

#rosdep
rosdep update
rosdep install --from-paths /ros/application_ws/src --ignore-src -r -y

# Optional: source overlay if needed
if [ -f "/ros/application_ws/install/setup.bash" ]; then
  source /ros/application_ws/install/setup.bash
fi

# Build
cd /ros/application_ws
colcon build --symlink-install
touch /tmp/build_done

# Keep container alive interactively if no other command was passed
if [ "$#" -eq 0 ]; then
  exec bash
else
  exec "$@"
fi
