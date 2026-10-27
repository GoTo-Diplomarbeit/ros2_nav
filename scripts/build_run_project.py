#!/usr/bin/env python3
import subprocess
import sys

# -- Run the command
# check=True stops with an error if the launch fails
try:
    subprocess.run(
        ["bash", "-c",
         "source /opt/ros/jazzy/setup.bash && "
         "source /unitree_go2_sim/install/setup.bash && "
         "colcon build --symlink-install --packages-select goto && "
         "source install/setup.bash && "
         "ros2 launch goto bringup.launch.py log_level:=warn"
         ],
        check=True)
except KeyboardInterrupt:
    sys.exit(0)
