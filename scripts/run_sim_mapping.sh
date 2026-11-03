#!/usr/bin/env bash
# =============================================================================
# run_sim_mapping.sh - MAP a Gazebo world, from ONE terminal
#
#   1. Gazebo simulator with the Go2 (CHAMP controller, sensors, EKFs)
#   2. SLAM + rviz          (RTAB-Map by default, or slam_toolbox) - NO Nav2
#   3. Keyboard teleop, right here in this terminal - you drive, the map grows
#
# To NAVIGATE on a finished map use scripts/run_sim_navigation.sh instead.
# Why two scripts: while mapping, RTAB-Map rebuilds the whole map after every loop
# closure (measured: up to 30 s on a 150-keyframe map). Nav2 driving at the same time
# then runs blind on drifting odometry and damages the map. Navigating on a fixed,
# finished map has none of that.
#
# Usage:
#   scripts/run_sim_mapping.sh [slam] [world] [--new]
#   scripts/run_sim_mapping.sh                  # RTAB-Map: 3D map + 2D map
#   scripts/run_sim_mapping.sh slam_toolbox     # the old 2D-only pipeline (map is NOT saved,
#                                               # and it still starts Nav2)
#   scripts/run_sim_mapping.sh rtabmap school_with_person   # another Gazebo world
#   scripts/run_sim_mapping.sh rtabmap default --new        # start this world's map over
#
#   world = a Gazebo world from ~/go2_ws/src/unitree_go2_ros2/unitree_go2_description/worlds/
#           (file name without .sdf), or a full path to an .sdf file. Default: "default".
#
# Maps (RTAB-Map only): ONE map per world, in ~/maps/<world>.db, kept between runs.
#   - First run of a world: a new map is created.
#   - Every later run CONTINUES that map: RTAB-Map loads it, first finds the robot on it
#     (localization mode - nothing is added to the map yet), then switches to mapping
#     mode by itself, and new areas are added to the same map.
#   - Before each run the map is copied to ~/maps/<world>.backup.db (the state before
#     this run), in case a run goes wrong.
#   - --new moves the old map aside to ~/maps/<world>_old_<date>.db (never deleted).
#
# Stop EVERYTHING: press Ctrl+C in this terminal (while teleop is running).
#
# The simulator and SLAM print a lot, so their output goes to log files instead
# of this terminal (the script prints where). Watch one live with: tail -f <file>
#
# Shared settings (switched-off sensors, ...) are in scripts/sim_common.sh.
#
# Pitfalls:
#   - Only one copy may run at a time (this script or run_sim_navigation.sh). The
#     script refuses to start if it finds one already running.
#   - The teleop only receives keys while THIS terminal window has focus.
#   - "2D Goal Pose" in rviz does nothing here - Nav2 is not running.
#   - Never drag the robot in the Gazebo window: the odometry doesn't notice the jump
#     and RTAB-Map draws the new surroundings at the wrong place.
#   - While the robot is not found yet on a continued map, driving adds NOTHING to the
#     map. The script prints when it was found. It assumes the robot starts where the map
#     started (x=0, y=0 - where the sim spawns it). If it is not found, click
#     "2D Pose Estimate" in rviz on the robot's real position.
#   - Started by hand (ros2 launch goto navigation_rtabmap.launch.py) RTAB-Map still
#     uses ~/.ros/rtabmap.db and DELETES it at startup - only these scripts keep maps.
#   - New launch/config files still need a build first:
#       PYTHONNOUSERSITE=1 colcon build --symlink-install --packages-select goto
#     (PYTHONNOUSERSITE=1 - see scripts/build_run_project.sh)
# =============================================================================

# ---- Settings you may want to change ----------------------------------------
# Slow, smooth driving gives the cleanest map; fast spins on the spot are where the
# odometry drifts. CHAMP caps the robot at 0.3 m/s and 0.5 rad/s anyway
# (max_linear_velocity_x / max_angular_velocity_z in the sim's config/gait/gait.yaml),
# so higher values here would not make it faster.
TELEOP_SPEED=0.3      # m/s
TELEOP_TURN=0.5       # rad/s
# -----------------------------------------------------------------------------

# Read the arguments: "--new" may stand anywhere, the rest are [slam] [world]
NEW_MAP=false
positional=()
for argument in "$@"; do
    if [[ "$argument" == "--new" ]]; then NEW_MAP=true; else positional+=("$argument"); fi
done
SLAM="${positional[0]:-rtabmap}"
WORLD="${positional[1]:-default}"
if [[ "$SLAM" != "rtabmap" && "$SLAM" != "slam_toolbox" ]]; then
    echo "Unknown SLAM backend '$SLAM'. Use: rtabmap (default) or slam_toolbox"
    exit 1
fi

# Shared helpers and settings (start_simulator, start_rtabmap, ...)
source "$(dirname "${BASH_SOURCE[0]}")/sim_common.sh"
setup_run

cleanup() {
    trap - EXIT INT TERM HUP
    stop_everything
    if [[ "$SLAM" == "rtabmap" && -n "$SLAM_ID" ]]; then
        echo "Your map is saved in $MAP_FILE - the next run with world '$WORLD' continues it."
        echo "Navigate on it with:  scripts/run_sim_navigation.sh $WORLD"
    fi
}
trap cleanup EXIT             # runs however the script ends
trap 'exit 130' INT TERM HUP  # Ctrl+C / closed window -> exit -> cleanup

# Called (in the background) once RTAB-Map has found the robot on a continued map:
# switch from localization mode (only searching) to mapping mode (adding to the map).
switch_to_mapping() {
    echo
    if ros2 service call /rtabmap/set_mode_mapping std_srvs/srv/Empty > /dev/null 2>&1; then
        echo ">>> Robot found on the map '$MAP_NAME' - mapping continues, new areas are now added."
    else
        echo ">>> Robot found, but switching to mapping failed. Nothing new is added to the map."
        echo "    Try by hand: ros2 service call /rtabmap/set_mode_mapping std_srvs/srv/Empty"
    fi
}

# ---- Prepare this world's map file (RTAB-Map only) ------------------------------
# Done before anything starts, so a problem here costs no waiting time.
RTABMAP_LOCALIZATION=false
if [[ "$SLAM" == "rtabmap" ]]; then
    mkdir -p "$MAP_DIR"
    if [[ "$NEW_MAP" == true && -f "$MAP_FILE" ]]; then
        old_map_file="$MAP_DIR/${MAP_NAME}_old_$(date +%Y-%m-%d_%H-%M).db"
        mv "$MAP_FILE" "$old_map_file"
        echo "--new: the old map was moved to $old_map_file"
    fi
    if [[ -f "$MAP_FILE" ]]; then
        # Continue the map: first find the robot on it (localization mode)
        RTABMAP_LOCALIZATION=true
        cp "$MAP_FILE" "$MAP_DIR/$MAP_NAME.backup.db"
        echo "Map: continuing $MAP_FILE (state before this run: $MAP_NAME.backup.db)"
    else
        echo "Map: none for world '$WORLD' yet - creating $MAP_FILE"
    fi
fi

# ---- 1. Simulator --------------------------------------------------------------
# rviz: RTAB-Map's launch brings its own rviz (3D view); slam_toolbox uses the sim's.
if [[ "$SLAM" == "rtabmap" ]]; then sim_rviz=false; else sim_rviz=true; fi
start_simulator "$sim_rviz" "[1/3]"

# ---- 2. SLAM + rviz ---------------------------------------------------------------
if [[ "$SLAM" == "rtabmap" ]]; then
    # use_nav2:=false - mapping only (see the top of this file). That also leaves out
    # the /scan converter, which only Nav2 needs.
    # localization:=true on a continued map, see the top of this file.
    start_rtabmap "[2/3]" "RTAB-Map + rviz" use_nav2:=false localization:=$RTABMAP_LOCALIZATION
else
    slam_log="$LOG_DIR/slam_toolbox.log"
    echo "[2/3] Starting slam_toolbox + Nav2     (log: $slam_log)"
    SLAM_ID=$(start_in_background "$slam_log" ros2 launch goto navigation.launch.py)
    echo "      waiting for the first map ..."
    wait_for_message /map 60 "$SLAM_ID" || { echo "      No /map after 60 s. See $slam_log"; exit 1; }
fi
echo "      mapping is running."
if [[ "$RTABMAP_LOCALIZATION" == true ]]; then
    echo "      continued map: looking for the robot on it first ..."
    watch_localization switch_to_mapping &
    WATCHER_ID=$!
fi

# ---- 3. Teleop (foreground) -----------------------------------------------------
# Runs in THIS terminal so it receives your key presses. When you press Ctrl+C,
# teleop ends, the script reaches its end, and cleanup stops everything else.
echo "[3/3] Teleop - keep this window focused. Ctrl+C here stops EVERYTHING."
echo
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -p speed:=$TELEOP_SPEED -p turn:=$TELEOP_TURN
