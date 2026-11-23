#!/usr/bin/env bash
# Maps a floor on its own with slam_toolbox (in Docker): the same explorer and Nav2 settings as
# build_run_sim_autonomous_mapping.sh (RTAB-Map), but slam_toolbox builds the 2D map from /scan.
# Both versions can be used and compared; this one never touches the RTAB-Map files.
#
#   bash scripts/build_run_sim_autonomous_mapping_slam_toolbox.sh [world]
#   SERVICE=release-nvidia bash scripts/build_run_sim_autonomous_mapping_slam_toolbox.sh   # NVIDIA GPU
#   GUI=false bash scripts/<this script>   # Gazebo without its window: less CPU, rviz still shows the robot
#
# world: file name without .sdf of a world in worlds/ (this project) or of the simulator (default: "default").
# Always a new map (slam_toolbox can't continue RTAB-Map's .db). Result in maps/:
#   <world>_slam_toolbox.png + .yaml        the 2D map, like the RTAB-Map version makes it
#   <world>_slam_toolbox.posegraph + .data  slam_toolbox's own file, to continue or localize on it later
# An earlier result with the same name is moved to <world>_slam_toolbox_old_<date>.* first.
#
# Keys in this terminal (you can take over at any time):
#   p  pause / resume exploring     i j l ,  drive by hand (pauses exploring first)
#   k  stop moving                  x or Ctrl+C  stop and save the map
# When no frontier is left the robot stops and waits: press x to save, or drive to areas it missed and press p.
#
# Not done for this version: continuing a map, navigation on it (build_run_sim_navigation.sh needs RTAB-Map's .db).
# Stop with x or Ctrl+C here - not with "docker compose down".

TELEOP_SPEED=0.3   # m/s   - CHAMP caps the robot at 0.3 m/s and 0.5 rad/s anyway (gait.yaml)
TELEOP_TURN=0.5    # rad/s

WORLD=default
MAP_IMAGE_SAVED=false
EXPLORER_ID=""
for argument in "$@"; do
    case "$argument" in
        -*) echo "Unknown option '$argument'. Usage: bash scripts/build_run_sim_autonomous_mapping_slam_toolbox.sh [world]"; exit 1 ;;
        *) WORLD="$argument" ;;
    esac
done

source "$(dirname "${BASH_SOURCE[0]}")/sim_common.sh"
run_in_docker "$@"   # on the host: start the container and run this script in it
setup_run

SLAM_NAME="slam_toolbox"                  # for the messages of the shared stop code
MAP_NAME="${MAP_NAME}_slam_toolbox"       # its own files, next to RTAB-Map's maps/<world>.*

# Saves slam_toolbox's own map file (pose graph) next to the image, while slam_toolbox still runs
save_pose_graph() {
    [[ -n "$RTABMAP_ID" ]] && kill -0 "$RTABMAP_ID" 2>/dev/null || return 0
    echo "Saving the pose graph ..."
    if timeout 60 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
        "{filename: '$MAP_DIR/$MAP_NAME'}" > "$LOG_DIR/serialize_map.log" 2>&1 && grep -q "result=0" "$LOG_DIR/serialize_map.log"; then
        echo "Pose graph saved: maps/$MAP_NAME.posegraph + .data"
    else
        echo "Saving the pose graph failed (log: $LOG_DIR/serialize_map.log)"
    fi
}

# The hooks of stop_on_exit in sim_common.sh
before_stop() { stop_session "$EXPLORER_ID"; save_pose_graph; save_map_image; }
after_stop() {
    [[ -z "$RTABMAP_ID" ]] && return   # the run ended before slam_toolbox started
    [[ $1 != 0 ]] && echo "Check the map: slam_toolbox did not stop cleanly."
    [[ "$MAP_IMAGE_SAVED" == true ]] && echo "Map: maps/$MAP_NAME.png + maps/$MAP_NAME.yaml"
}

# Exploration settings: nav2_params.yaml and slam_params.yaml plus their *_exploration_overrides.yaml, merged into
# this run's log folder. Done first: if it fails, nothing has been touched yet.
GOTO_CONFIG="$(ros2 pkg prefix goto)/share/goto/config"
NAV2_PARAMS="$LOG_DIR/nav2_params_exploration.yaml"
python3 -m goto.node_utils.merge_params "$GOTO_CONFIG/nav2_params.yaml" "$GOTO_CONFIG/nav2_exploration_overrides.yaml" \
    "$NAV2_PARAMS" || { echo "Could not merge the Nav2 settings for exploration"; exit 1; }
# The same for slam_toolbox (config/slam_exploration_overrides.yaml)
SLAM_PARAMS="$LOG_DIR/slam_params_exploration.yaml"
python3 -m goto.node_utils.merge_params "$GOTO_CONFIG/slam_params.yaml" "$GOTO_CONFIG/slam_exploration_overrides.yaml" \
    "$SLAM_PARAMS" || { echo "Could not merge the slam_toolbox settings for exploration"; exit 1; }

# A result of an earlier run is kept: the map saver would overwrite it
mkdir -p "$MAP_DIR"
old_stamp="$(date +%Y-%m-%d_%H-%M-%S)"
for extension in png yaml posegraph data; do
    [[ -f "$MAP_DIR/$MAP_NAME.$extension" ]] || continue
    mv "$MAP_DIR/$MAP_NAME.$extension" "$MAP_DIR/${MAP_NAME}_old_$old_stamp.$extension"
    echo "The earlier maps/$MAP_NAME.$extension was moved to maps/${MAP_NAME}_old_$old_stamp.$extension"
done

start_simulator

echo "[2/3] Starting slam_toolbox + rviz + Nav2 (log: $LOG_DIR/slam_toolbox.log)"
RTABMAP_ID=$(start_in_background "$LOG_DIR/slam_toolbox.log" ros2 launch goto navigation.launch.py \
    nav2_params:="$NAV2_PARAMS" slam_params:="$SLAM_PARAMS" rviz:=true rviz_config:=autonomous_mapping.rviz)
echo "      waiting for the first map ..."
wait_for_message /map 60 "$RTABMAP_ID"
case $? in
    0) ;;
    2) echo "      The launch crashed:"; tail -15 "$LOG_DIR/slam_toolbox.log"; exit 1 ;;
    *) echo "      No map after a minute. See $LOG_DIR/slam_toolbox.log"; exit 1 ;;
esac

# wait_for_rtabmap:=false - slam_toolbox sends no /info, which the explorer waits for with RTAB-Map
EXPLORER_ID=$(start_in_background "$LOG_DIR/explorer.log" \
    ros2 run goto autonomous_mapping_node --ros-args -p wait_for_rtabmap:=false)

echo "[3/3] Keys (below) - keep this window focused. x or Ctrl+C stops everything and saves the map."
ros2 run goto autonomous_mapping_keys --ros-args -p speed:=$TELEOP_SPEED -p turn:=$TELEOP_TURN
