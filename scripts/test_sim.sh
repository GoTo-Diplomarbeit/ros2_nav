cd ~/

sleep 40
ros2 topic pub --once /planner_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"

