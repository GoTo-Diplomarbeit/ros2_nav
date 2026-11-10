#!/usr/bin/env bash
# Maps a Gazebo world with RTAB-Map while you drive the robot with the keyboard (in Docker).
#
#   bash scripts/build_run_sim_mapping.sh [world] [--new]
#   SERVICE=release-nvidia bash scripts/build_run_sim_mapping.sh   # render on the NVIDIA GPU
#
# world: a world of the simulator, file name without .sdf (default: "default").
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

# The hooks of stop_on_exit in sim_common.sh.
# before_stop saves the map image while RTAB-Map still runs - but only if this run really built
# the map: a new map, or a continued one after the robot was found (before that RTAB-Map only
# localizes, and /map may be incomplete).
before_stop() {
    if [[ "$CONTINUE_MAP" == false || -f "$LOG_DIR/robot_found" ]]; then
        save_map_image
    else
        echo "The robot was never found on the map - the map image is left as it was."
    fi
}
after_stop() {
    [[ -z "$RTABMAP_ID" ]] && return   # the run ended before RTAB-Map started
    if [[ $1 == 0 ]]; then
        echo "Map saved in maps/$MAP_NAME.db. Navigate on it: bash scripts/build_run_sim_navigation.sh $WORLD"
    elif [[ "$CONTINUE_MAP" == true ]]; then
        echo "The map as it was before this run: maps/$MAP_NAME.backup.db"
    fi
    if [[ "$MAP_IMAGE_SAVED" == true && -f "$MAP_DIR/${MAP_NAME}_rooms.yaml" ]]; then
        echo "The map changed - check the rooms still fit: bash scripts/build_run_define_rooms.sh $WORLD"
    fi
}

# Called once RTAB-Map has found the robot on the continued map
switch_to_mapping() {
    if ros2 service call /rtabmap/set_mode_mapping std_srvs/srv/Empty > /dev/null 2>&1; then
        touch "$LOG_DIR/robot_found"   # for before_stop: the map is being extended now
        echo ">>> Robot found on the map - new areas are added from now on."
    else
        echo ">>> Robot found, but switching to mapping failed - nothing new is added. Try by hand:"
        echo "    ros2 service call /rtabmap/set_mode_mapping std_srvs/srv/Empty"
    fi
}

# Prepare the map file
mkdir -p "$MAP_DIR"
if [[ "$NEW_MAP" == true && -f "$MAP_FILE" ]]; then
    old_map="${MAP_NAME}_old_$(date +%Y-%m-%d_%H-%M-%S).db"
    mv "$MAP_FILE" "$MAP_DIR/$old_map"
    rm -f "$MAP_DIR/$MAP_NAME.backup.db"   # belonged to the old map, which is kept in $old_map
    echo "--new: the old map was moved to maps/$old_map"
    # The rooms are in the old map's coordinates, so they go with it. The image is made new at the end.
    if [[ -f "$MAP_DIR/${MAP_NAME}_rooms.yaml" ]]; then
        mv "$MAP_DIR/${MAP_NAME}_rooms.yaml" "$MAP_DIR/${old_map%.db}_rooms.yaml"
        echo "       its rooms were moved to maps/${old_map%.db}_rooms.yaml - define the rooms again"
    fi
    rm -f "$MAP_DIR/$MAP_NAME.png" "$MAP_DIR/$MAP_NAME.yaml"
fi
if [[ -f "$MAP_FILE" ]]; then
    CONTINUE_MAP=true
    cp "$MAP_FILE" "$MAP_DIR/$MAP_NAME.backup.db"
    echo "Continuing maps/$MAP_NAME.db"
else
    echo "New map: maps/$MAP_NAME.db"
fi

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
