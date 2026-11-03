#!/usr/bin/env bash
# Triggers a photo capture in the vision node and prints the result.
# The mission state must be REGISTER_PERSON (see change_to_register_person_state.sh),
# otherwise the vision node rejects the trigger.
# Result: the saved image path on success, empty string on failure.
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../../.." && pwd)"

# Max time to wait for a result (countdown 5 s + person timeout 10 s + margin)
result_timeout="${RESULT_TIMEOUT:-25}"

source "$(dirname -- "${BASH_SOURCE[0]}")/_find_service.sh"

docker compose -f "$repo_root/docker-compose.yaml" exec -T "$SERVICE" bash -c '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /unitree_go2_sim/install/setup.bash
    cd /ws
    source install/setup.bash

    # listen for the result before triggering so it is not missed
    timeout "$1" ros2 topic echo --once /vision/picture_result std_msgs/msg/String &
    echo_pid=$!
    sleep 1

    ros2 topic pub --once /vision/take_picture std_msgs/msg/Empty "{}"

    if ! wait "$echo_pid"; then
        echo "No result on /vision/picture_result within $1 s" >&2
        exit 1
    fi
' bash "$result_timeout"
