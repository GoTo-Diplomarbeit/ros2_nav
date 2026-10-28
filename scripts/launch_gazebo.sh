#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

docker compose up -d --remove-orphans release
docker compose exec release bash -c '
    cd /ws
    source /opt/ros/jazzy/setup.bash
    source /unitree_go2_sim/install/setup.bash
    exec ros2 launch unitree_go2_sim unitree_go2_launch.py
'
