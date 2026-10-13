cd ~/

sleep 40
ros2 topic pub --once /planner_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"

sleep 10
# 1. Kurve starten (0.5 m/s vorwärts, 0.5 rad/s Drehung)
ros2 topic pub --once /planner_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1}, angular: {z: 0.0}}"

# 2. Warten, bis die 90 Grad erreicht sind
sleep 1.57

# 3. Roboter stoppen
ros2 topic pub --once /planner_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.0}}"

sleep 5
