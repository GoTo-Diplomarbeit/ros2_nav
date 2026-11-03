#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
state="${1:-IDLE}"

docker compose -f "$repo_root/docker-compose.yaml" exec -T release bash -c '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /unitree_go2_sim/install/setup.bash
    cd /ws
    source install/setup.bash
    exec ros2 topic pub --once /mission_state_request std_msgs/msg/String "{data: $1}"
' bash "$state"
