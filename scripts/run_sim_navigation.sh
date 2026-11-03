#!/usr/bin/env bash
# =============================================================================
# run_sim_navigation.sh - NAVIGATE on a saved map, from ONE terminal
#
#   1. Gazebo simulator with the Go2 (only the sensors navigation needs)
#   2. RTAB-Map in LOCALIZATION mode + rviz + Nav2:
#        RTAB-Map loads ~/maps/<world>.db, finds the robot on it and keeps tracking
#        it - but NEVER changes the map. Nav2 plans and drives on that map.
#   3. Waits here until you press Ctrl+C
#
# Make the map first with scripts/run_sim_mapping.sh (same world name).
# Why a separate script: while mapping, RTAB-Map rebuilds the whole map after every
# loop closure (up to 30 s measured), and Nav2 then drives blind. On a fixed map
# nothing is rebuilt, and a bad drive can't damage the map.
#
# Usage:
#   scripts/run_sim_navigation.sh [world]
#   scripts/run_sim_navigation.sh                      # world "default" -> ~/maps/default.db
#   scripts/run_sim_navigation.sh school_with_person   # -> ~/maps/school_with_person.db
#
# Then in rviz: "2D Goal Pose" -> click + drag on a WHITE (known, free) spot of the map.
#
# Stop EVERYTHING: press Ctrl+C in this terminal.
#
# Shared settings (switched-off sensors, ...) are in scripts/sim_common.sh.
#
# Pitfalls:
#   - Only one copy may run at a time (this script or run_sim_mapping.sh).
#   - RTAB-Map assumes the robot starts where the map started (x=0, y=0 - where the sim
#     spawns it). If the script says it can't find the robot, click "2D Pose Estimate"
#     in rviz on the robot's real position. Goals sent before it is found are planned
#     from the wrong position.
#   - Areas that were never mapped are grey = unknown, and Nav2 never plans through
#     them ("Failed to create plan"). Map them first with run_sim_mapping.sh.
#   - Never drag the robot in the Gazebo window - the odometry doesn't notice the jump.
#   - New obstacles (not on the map) are still avoided: Nav2 sees them live through
#     /scan. They just aren't saved into the map.
# =============================================================================

WORLD="${1:-default}"

# Shared helpers and settings (start_simulator, start_rtabmap, ...)
source "$(dirname "${BASH_SOURCE[0]}")/sim_common.sh"
setup_run

# Navigation needs a finished map - check before anything starts
if [[ ! -f "$MAP_FILE" ]]; then
    echo "No map for world '$WORLD' ($MAP_FILE)."
    echo "Make one first:  scripts/run_sim_mapping.sh rtabmap $WORLD"
    exit 1
fi
echo "Map: navigating on $MAP_FILE (it is only read, never changed)"

cleanup() {
    trap - EXIT INT TERM HUP
    stop_everything
}
trap cleanup EXIT             # runs however the script ends
trap 'exit 130' INT TERM HUP  # Ctrl+C / closed window -> exit -> cleanup

# Called (in the background) once RTAB-Map has found the robot on the map
announce_ready() {
    echo
    echo ">>> Robot found on the map '$MAP_NAME' - ready. Set goals in rviz with \"2D Goal Pose\"."
}

# ---- 1. Simulator --------------------------------------------------------------
# rviz:=false - RTAB-Map's launch brings its own rviz with the map view
start_simulator false "[1/3]"

# ---- 2. RTAB-Map (localization) + rviz + Nav2 ----------------------------------------
# localization:=true and it stays true: no mapping-mode switch here, so the map file
# is never extended. (RTAB-Map only remembers its last position in the file.)
start_rtabmap "[2/3]" "RTAB-Map (localization) + rviz + Nav2" use_nav2:=true localization:=true
echo "      looking for the robot on the map ..."
watch_localization announce_ready &
WATCHER_ID=$!

# ---- 3. Wait (foreground) ---------------------------------------------------------------
# Nothing to type here - goals come from rviz. The script just stays alive so that
# Ctrl+C in this terminal stops everything. If the simulator or RTAB-Map dies on its
# own, the script notices and stops the rest too.
echo "[3/3] Running. Ctrl+C here stops EVERYTHING."
while kill -0 "$SIM_ID" 2>/dev/null && kill -0 "$SLAM_ID" 2>/dev/null; do
    sleep 1
done
echo "The simulator or RTAB-Map stopped by itself - see the logs in $LOG_DIR"
