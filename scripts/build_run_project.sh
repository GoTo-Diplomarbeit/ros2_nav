#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

docker compose up -d --remove-orphans release
docker compose exec release bash -c '
    set -e
    cd /ws
    source /opt/ros/jazzy/setup.bash
    source /unitree_go2_sim/install/setup.bash
    colcon build --symlink-install --packages-select goto
    source install/setup.bash
    exec ros2 launch goto bringup.launch.py log_level:=warn
'
