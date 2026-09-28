source ~/go2_ws/install/setup.bash
cd ~/Documents/ros2_nav

colcon build --symlink-install --packages-select go2_nav
source install/setup.bash

ros2 launch go2_nav bringup.launch.py
