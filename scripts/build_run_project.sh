#!/usr/bin/env bash
# Build go2_nav and start the project's own nodes (bringup.launch.py).
# Start the simulator first (scripts/launch_gazebo_with_lidar.sh).

# Stop at the first failing command. Without this, a failing "cd" just carried on
# in whatever folder the script was started from, built the project THERE, and
# then launched the old, stale build - e.g. "executable 'camera_node' not found".
set -e

source ~/go2_ws/install/setup.bash

# The repo root, found relative to this script, so it works on every computer.
# (A hard-coded path like /home/<name>/... only exists on one person's machine.)
cd "$(dirname "${BASH_SOURCE[0]}")/.."

# PYTHONNOUSERSITE=1: build with Ubuntu's setuptools, not the newer one that pip puts
# into ~/.local together with PyTorch (step 4b). setuptools >= 80 dropped an option
# 'colcon build --symlink-install' needs (error: option --uninstall not recognized).
# Only affects the build - the nodes still find ~/.local packages when they run.
PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select goto
source install/setup.bash

ros2 launch goto bringup.launch.py log_level:=warn
