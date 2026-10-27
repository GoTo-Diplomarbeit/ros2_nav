#!/usr/bin/env python3
import subprocess
import argparse
import sys

# Setup default launch command
launch_command = "ros2 launch unitree_go2_sim unitree_go2_launch.py"

# Create a command line argument parser
argparser = argparse.ArgumentParser()
argparser.add_argument("--disable_lidar", type=bool, default=False)
argparser.add_argument("--disable_rviz", type=bool, default=False)

# Parse the arguments from the console
args = argparser.parse_args()

# -- Check arguments
# Disable lidar
if args.disable_lidar:
    launch_command = f"{launch_command} disable_velodyne_lidar:=true disable_d455:=true"

# Disable rviz
if args.disable_rviz:
    launch_command = f"{launch_command} rviz:=false"

# -- Run the command
# check=True stops with an error if the launch fails
try:
    subprocess.run(
        ["bash", "-c",
         "source /opt/ros/jazzy/setup.bash && "
         "source /unitree_go2_sim/install/setup.bash && " + launch_command
         ],
        check=True)
except KeyboardInterrupt:
    sys.exit(0)
