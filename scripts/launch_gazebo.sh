#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."

# Override with SERVICE=release-nvidia for GPU rendering (see README)
SERVICE=${SERVICE:-release}

docker compose up -d --remove-orphans "$SERVICE"
# Extra arguments are forwarded to the launch file, e.g. world:=/ws/path/to/world.sdf
docker compose exec "$SERVICE" bash -c '
    cd /ws
    source /opt/ros/jazzy/setup.bash
    source /unitree_go2_sim/install/setup.bash
    exec ros2 launch unitree_go2_sim unitree_go2_launch.py "$@"
' bash "$@"
