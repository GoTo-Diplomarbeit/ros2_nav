#!/usr/bin/env bash
# Maps a Gazebo world with RTAB-Map while you drive the robot with the keyboard (in Docker).
#
#   bash scripts/build_run_sim_mapping.sh [world] [--new]
#   SERVICE=release-nvidia bash scripts/build_run_sim_mapping.sh   # render on the NVIDIA GPU
#
# world: file name without .sdf of a world in worlds/ (this project) or of the simulator (default: "default").
# The map is saved in maps/<world>.db. If it already exists, this run continues it: RTAB-Map
# first finds the robot on it, then adds new areas. maps/<world>.backup.db is the state
# before the run. --new moves the old map to maps/<world>_old_<date>.db and starts over.
#
# Nav2 isn't started here: after a loop closure RTAB-Map rebuilds the map (up to 30 s) and
# Nav2 would drive blind meanwhile. Navigate on the finished map with build_run_sim_navigation.sh.
#
# Stop with Ctrl+C here - not with "docker compose down", RTAB-Map saves the map when it stops.
# Teleop only gets keys while this terminal has focus. Don't drag the robot in Gazebo: the
# odometry doesn't notice and RTAB-Map draws the new area at the wrong place.

TELEOP_SPEED=0.3   # m/s   - CHAMP caps the robot at 0.3 m/s and 0.5 rad/s anyway (gait.yaml)
TELEOP_TURN=0.5    # rad/s

WORLD=default
NEW_MAP=false
CONTINUE_MAP=false
MAP_IMAGE_SAVED=false
for argument in "$@"; do
    case "$argument" in
        --new) NEW_MAP=true ;;
        -*) echo "Unknown option '$argument'. Usage: bash scripts/build_run_sim_mapping.sh [world] [--new]"; exit 1 ;;
        *) WORLD="$argument" ;;
    esac
done

source "$(dirname "${BASH_SOURCE[0]}")/sim_common.sh"
run_in_docker "$@"   # on the host: start the container and run this script in it
setup_run

# The hooks of stop_on_exit in sim_common.sh
before_stop() { mapping_before_stop; }
after_stop() { mapping_after_stop "$1"; }

prepare_map_file

start_simulator
# A continued map starts in localization mode: RTAB-Map only looks for the robot and adds
# nothing until switch_to_mapping. (Starting in mapping mode would begin a new, unconnected map.)
start_rtabmap "RTAB-Map + rviz" use_nav2:=false localization:=$CONTINUE_MAP
if [[ "$CONTINUE_MAP" == true ]]; then
    echo "      looking for the robot on the map ..."
    watch_localization switch_to_mapping &
    WATCHER_ID=$!
fi

echo "[3/3] Teleop - keep this window focused. Ctrl+C stops everything."
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=$TELEOP_SPEED -p turn:=$TELEOP_TURN
