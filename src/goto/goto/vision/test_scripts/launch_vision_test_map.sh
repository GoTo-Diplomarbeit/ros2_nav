#!/usr/bin/env bash
# Starts the Go2 simulator in the vision test world (../test_map/vision_test_map.sdf).
# Uses scripts/launch_gazebo.sh, so SERVICE=release-nvidia works here too.
# Extra arguments are passed on to the launch file, e.g.:
#   bash launch_vision_test_map.sh rviz:=false
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../../.." && pwd)"

# The repo is mounted at /ws inside the container
world_path="/ws/src/goto/goto/vision/test_map/vision_test_map.sdf"

exec "$repo_root/scripts/launch_gazebo.sh" world:="$world_path" "$@"
