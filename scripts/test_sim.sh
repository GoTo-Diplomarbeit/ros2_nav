docker compose exec release bash -c '    set -e
    cd /ws
    source /opt/ros/jazzy/setup.bash
    source /unitree_go2_sim/install/setup.bash
    ros2 topic pub --once /planner_cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}"
    '
