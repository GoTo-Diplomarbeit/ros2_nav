#!/usr/bin/env bash
# Navigates on a saved map with Nav2 (in Docker).
#
#   bash scripts/build_run_sim_navigation.sh [world]
#   SERVICE=release-nvidia bash scripts/build_run_sim_navigation.sh   # render on the NVIDIA GPU
#
# RTAB-Map loads maps/<world>.db in localization mode: it finds the robot on the map and tracks
# it, but adds nothing to the map. Make the map first with build_run_sim_mapping.sh.
# Set goals in rviz with "2D Goal Pose" on known (white) areas - Nav2 doesn't plan through
# unmapped (grey) ones. New obstacles are still avoided, they just aren't saved in the map.
#
# The robot is assumed to start where the map started (where the sim spawns it). If it isn't
# found, click "2D Pose Estimate" in rviz on its real position. Stop with Ctrl+C here.

if (( $# > 1 )) || [[ "${1:-}" == -* ]]; then
    echo "Usage: bash scripts/build_run_sim_navigation.sh [world]"
    exit 1
fi
WORLD="${1:-default}"

source "$(dirname "${BASH_SOURCE[0]}")/sim_common.sh"
run_in_docker "$@"   # on the host: start the container and run this script in it
setup_run

if [[ ! -f "$MAP_FILE" ]]; then
    echo "No map for world '$WORLD' yet. Make one first: bash scripts/build_run_sim_mapping.sh $WORLD"
    exit 1
fi

announce_ready() {
    echo ">>> Robot found on the map - set goals in rviz with \"2D Goal Pose\"."
}

echo "Navigating on maps/$MAP_NAME.db"
start_simulator
start_rtabmap "RTAB-Map (localization) + rviz + Nav2" use_nav2:=true localization:=true
echo "      looking for the robot on the map ..."
watch_localization announce_ready &
WATCHER_ID=$!

echo "[3/3] Running. Ctrl+C stops everything."
# Stay alive until Ctrl+C - or until the simulator or RTAB-Map dies by itself
while kill -0 "$SIM_ID" 2>/dev/null && kill -0 "$RTABMAP_ID" 2>/dev/null; do
    sleep 1
done
echo "The simulator or RTAB-Map stopped by itself - see $LOG_DIR"
