#!/usr/bin/env bash
# Names the rooms on a saved map, for the app's room selection (in Docker, no simulator needed).
#
#   bash scripts/build_run_define_rooms.sh [world]
#
# Opens the map image (maps/<world>.png, saved at the end of build_run_sim_mapping.sh) in a window.
# For each room: type its name here, click its corners in the window, then where the robot should
# stop and a point in the direction it should face. The rooms are saved in maps/<world>_rooms.yaml,
# in metres on the map, after every room. To delete a room, edit that file by hand.

if (( $# > 1 )) || [[ "${1:-}" == -* ]]; then
    echo "Usage: bash scripts/build_run_define_rooms.sh [world]"
    exit 1
fi
WORLD="${1:-default}"
MAP_NAME="$(basename "$WORLD" .sdf)"

source "$(dirname "${BASH_SOURCE[0]}")/sim_common.sh"
CHECK_FOR_RUNNING_SIM=false   # uses no ROS, so it may run next to a sim
run_in_docker "$@"   # on the host: start the container and run this script in it
trap give_files_back EXIT   # the container runs as root - hand the room file back to you

if [[ ! -f "$MAP_DIR/$MAP_NAME.yaml" ]]; then
    echo "No map image for world '$WORLD' yet. Map it first: bash scripts/build_run_sim_mapping.sh $WORLD"
    exit 1
fi

# Runs straight from the source code - it uses no ROS, so nothing has to be built
# (-B: write no __pycache__, it would belong to root in the source folder)
cd "$REPO_DIR/src/goto" && python3 -B -m goto.rooms.define_rooms "$MAP_DIR/$MAP_NAME.yaml" "$MAP_DIR/${MAP_NAME}_rooms.yaml"
