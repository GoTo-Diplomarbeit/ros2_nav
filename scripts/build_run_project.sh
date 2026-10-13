source ~/go2_ws/install/setup.bash
cd /home/gazebo-kilian/Diplomarbeit_DAWG/ros2_nav

colcon build --symlink-install --packages-select go2_nav
source install/setup.bash

ros2 launch go2_nav bringup.launch.py log_level:=warn
