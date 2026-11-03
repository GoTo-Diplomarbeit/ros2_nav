#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

# Override with SERVICE=release-nvidia for GPU rendering (see README)
SERVICE=${SERVICE:-release}

docker compose up -d --remove-orphans "$SERVICE"
docker compose exec "$SERVICE" bash -c '
    cd /ws
    source /opt/ros/jazzy/setup.bash
    source /unitree_go2_sim/install/setup.bash
    source /ws/install/setup.bash

    ros2 launch unitree_go2_sim unitree_go2_launch.py static_map_to_odom:=false rviz:=false &
    sleep 30

    ros2 launch goto navigation_rtabmap.launch.py &
    sleep 10

    ros2 run teleop_twist_keyboard teleop_twist_keyboard
'
