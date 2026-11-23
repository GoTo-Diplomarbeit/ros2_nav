#!/usr/bin/env bash
# Maps a floor on its own (in Docker): the robot drives to the edge of the known map again and again,
# until nothing is left to explore. RTAB-Map builds the map, Nav2 drives, autonomous_mapping_node decides where.
#
#   bash scripts/build_run_sim_autonomous_mapping.sh [world] [--new]
#   SERVICE=release-nvidia bash scripts/build_run_sim_autonomous_mapping.sh   # render on the NVIDIA GPU
#   GUI=false bash scripts/<this script>   # Gazebo without its window: less CPU, rviz still shows the robot
#
# world: file name without .sdf of a world in worlds/ (this project) or of the simulator (default: "default").
# The map is saved in maps/<world>.db. If it already exists, this run continues it: RTAB-Map first finds the
# robot on it, then the robot explores what is missing. maps/<world>.backup.db is the state before the run.
# --new moves the old map to maps/<world>_old_<date>.db and starts over.
#
# Keys in this terminal (you can take over at any time):
#   p  pause / resume exploring     i j l ,  drive by hand (pauses exploring first)
#   k  stop moving                  x or Ctrl+C  stop and save the map
# When no frontier is left the robot stops and waits: press x to save, or drive to areas it missed and press p.
#
# rviz shows no OctoMap here (rviz/autonomous_mapping.rviz): RTAB-Map only builds the 3D map while something
# subscribes to it, and building it froze RTAB-Map for up to a minute. Don't open rviz/rtabmap.rviz during this run.
#
# Wall distance: the explorer's robot_clearance_m, preferred_clearance_m and pullback_m (autonomous_mapping_node.py)
# and the inflation radius for exploration (config/nav2_exploration_overrides.yaml) are at the plain values, the same
# as before. Larger values (0.45 m clearance, 0.9 m inflation) keep the robot farther from walls but skip doors
# under 0.9 m: drive through those by hand (i j l ,).
#
# Stop with x or Ctrl+C here - not with "docker compose down", RTAB-Map saves the map when it stops.

TELEOP_SPEED=0.3   # m/s   - CHAMP caps the robot at 0.3 m/s and 0.5 rad/s anyway (gait.yaml)
TELEOP_TURN=0.5    # rad/s

WORLD=default
NEW_MAP=false
CONTINUE_MAP=false
MAP_IMAGE_SAVED=false
EXPLORER_ID=""
for argument in "$@"; do
    case "$argument" in
        --new) NEW_MAP=true ;;
        -*) echo "Unknown option '$argument'. Usage: bash scripts/build_run_sim_autonomous_mapping.sh [world] [--new]"; exit 1 ;;
        *) WORLD="$argument" ;;
    esac
done

source "$(dirname "${BASH_SOURCE[0]}")/sim_common.sh"
run_in_docker "$@"   # on the host: start the container and run this script in it
setup_run

# The hooks of stop_on_exit in sim_common.sh
before_stop() { stop_session "$EXPLORER_ID"; stop_drift_monitor; mapping_before_stop; }
after_stop() { mapping_after_stop "$1"; }

# A continued map: the explorer waits (it was started paused) until RTAB-Map has found the robot
start_exploring_when_found() {
    switch_to_mapping
    [[ -f "$LOG_DIR/robot_found" ]] || return   # switching failed: the explorer stays paused
    ros2 topic pub --once -w 1 /autonomous_mapping/command std_msgs/msg/String "{data: resume}" > /dev/null 2>&1
    echo ">>> The robot explores on its own now."
}

# Exploration keeps paths farther from walls: nav2_params.yaml plus config/nav2_exploration_overrides.yaml, merged into
# this run's log folder. Navigation (build_run_sim_navigation.sh) keeps using the unchanged nav2_params.yaml.
# Done first: if it fails, the map files have not been touched yet.
GOTO_CONFIG="$(ros2 pkg prefix goto)/share/goto/config"
NAV2_PARAMS="$LOG_DIR/nav2_params_exploration.yaml"
python3 -m goto.node_utils.merge_params "$GOTO_CONFIG/nav2_params.yaml" "$GOTO_CONFIG/nav2_exploration_overrides.yaml" \
    "$NAV2_PARAMS" || { echo "Could not merge the Nav2 settings for exploration"; exit 1; }

prepare_map_file

start_simulator
# A continued map starts in localization mode: RTAB-Map only looks for the robot and adds
# nothing until switch_to_mapping. (Starting in mapping mode would begin a new, unconnected map.)
start_rtabmap "RTAB-Map + rviz + Nav2" use_nav2:=true localization:=$CONTINUE_MAP \
    rviz_config:=autonomous_mapping.rviz nav2_params:="$NAV2_PARAMS"
EXPLORER_ID=$(start_in_background "$LOG_DIR/explorer.log" \
    ros2 run goto autonomous_mapping_node --ros-args -p start_paused:=$CONTINUE_MAP)
start_drift_monitor
if [[ "$CONTINUE_MAP" == true ]]; then
    echo "      looking for the robot on the map ..."
    watch_localization start_exploring_when_found &
    WATCHER_ID=$!
fi

echo "[3/3] Keys (below) - keep this window focused. x or Ctrl+C stops everything and saves the map."
ros2 run goto autonomous_mapping_keys --ros-args -p speed:=$TELEOP_SPEED -p turn:=$TELEOP_TURN
