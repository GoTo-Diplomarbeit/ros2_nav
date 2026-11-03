#!/usr/bin/env bash
# =============================================================================
# sim_common.sh - shared parts of run_sim_mapping.sh and run_sim_navigation.sh
#
# Not a script to run on its own: the two run scripts load it with "source", so both
# start the simulator, RTAB-Map and the shutdown in exactly the same way.
#
# What a script must do after sourcing it:
#   1. set WORLD, then call   setup_run            (ROS environments, log folder, map file)
#   2. set its own trap:      trap <its cleanup> EXIT; trap 'exit 130' INT TERM HUP
#   3. use start_simulator, start_rtabmap, watch_localization, stop_everything
# =============================================================================

# ---- Settings shared by both scripts ----------------------------------------
# Sensors switched off in the simulator. Mapping AND navigation only need the Velodyne
# (RTAB-Map + Nav2's /scan) plus IMU and leg data (odometry, can't be switched off).
# Every camera/lidar is rendered by Gazebo and bridged to ROS, which costs a lot of CPU.
# Only these two scripts turn them off; the simulator's defaults and the other scripts
# keep all sensors on.
#   disable_d455      RealSense depth camera (Micah's vision)
#   disable_camera    mono camera (video stream to the app)
#   disable_lidar_l1  Unitree L1 lidar (unused, the Velodyne is the mapping lidar)
# PITFALL: vision_node and camera_node get no images in a run started by these scripts.
# NOTE: "gui:=false" would save the Gazebo window's ~1-1.6 CPU cores, but the simulator
# launch currently ignores it (declared, never used) - the window always opens.
SIM_EXTRA_ARGS="disable_d455:=true disable_camera:=true disable_lidar_l1:=true"
RTABMAP_ATTEMPTS=3    # how often to restart RTAB-Map if it gets stuck at startup
MAP_DIR="$HOME/maps"  # one map per world: $MAP_DIR/<world>.db
# -----------------------------------------------------------------------------

# The repo root, found relative to THIS file, so it works wherever the repo is.
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Ids of what we started - the cleanup stops exactly these
SIM_ID=""
SLAM_ID=""
WATCHER_ID=""

# ---- Setup ----------------------------------------------------------------------

# Loads the ROS environments, refuses to run twice, creates the log folder and works
# out the map file. Needs WORLD to be set. Sets LOG_DIR, MAP_NAME, MAP_FILE.
setup_run() {
    LOG_DIR="$HOME/.ros/log/go2_run_$(date +%Y-%m-%d_%H-%M-%S)"
    mkdir -p "$LOG_DIR"

    # One map file per world, named after the world: "school_with_person" or
    # "/home/x/my_world.sdf" -> ~/maps/school_with_person.db / ~/maps/my_world.db
    MAP_NAME="$(basename "$WORLD" .sdf)"
    MAP_FILE="$MAP_DIR/$MAP_NAME.db"

    # Order matters: ROS itself, then the Go2 sim workspace, then this repo on top.
    local setup_file
    for setup_file in /opt/ros/jazzy/setup.bash "$HOME/go2_ws/install/setup.bash" "$REPO_DIR/install/setup.bash"; do
        if [[ ! -f "$setup_file" ]]; then
            echo "Missing $setup_file - has that workspace been built?"
            exit 1
        fi
        source "$setup_file"
    done

    # Only one copy may run: two simulators or two SLAM nodes both publish /map and
    # map->odom and the map breaks. The patterns are anchored to the START of each
    # command line ("^"), so they only match the real programs - not e.g. a terminal or
    # editor whose command line just happens to contain the words "gz sim".
    local already_running
    already_running=$(pgrep -af "^gz sim|^[^ ]*rtabmap_slam/rtabmap|^[^ ]*slam_toolbox/[^ ]*slam_toolbox_node|^[^ ]*nav2_bt_navigator/bt_navigator|^[^ ]*python3 [^ ]*/teleop_twist_keyboard( |$)")
    if [[ -n "$already_running" ]]; then
        echo "Something from a previous run is still running:"
        echo "$already_running" | sed 's/^/    /' | cut -c1-120
        echo "Stop it first (Ctrl+C in its terminal), otherwise two copies fight over the map."
        exit 1
    fi
}

# ---- Process helpers ------------------------------------------------------------

# Start a command in the background in its OWN session ("setsid"), with its output
# in a log file, and print its id.
# Why a session: everything the launch file starts stays in that session - even
# Gazebo's server and GUI, which put themselves into process groups of their own.
# So "stop every process in session X" catches all of it, and nothing survives as
# an orphan. The launch process's id is also the session id.
start_in_background() {
    local log_file=$1; shift
    setsid "$@" < /dev/null > "$log_file" 2>&1 &
    local id=$!
    # PITFALL: "&" returns a split second BEFORE setsid has created the new session.
    # Checking on the session in that gap fails, so wait until it really exists.
    for _ in $(seq 50); do
        [[ "$(ps -o sid= -p "$id" 2>/dev/null | tr -d ' ')" == "$id" ]] && break
        sleep 0.1
    done
    echo "$id"
}

# Wait until one message arrives on a topic. Returns 0 = got one, 1 = timed out,
# 2 = the launch we are waiting for has died, 130 = user pressed Ctrl+C.
wait_for_message() {
    local topic=$1 max_seconds=$2 launch_id=$3
    local deadline=$((SECONDS + max_seconds))
    while (( SECONDS < deadline )); do
        kill -0 "$launch_id" 2>/dev/null || return 2   # "kill -0" = only check it's alive
        # "ros2 topic echo --once" exits as soon as one message arrives. It fails
        # instantly while the topic doesn't exist yet, hence the loop.
        timeout 5 ros2 topic echo "$topic" --once --field header.stamp > /dev/null 2>&1
        local result=$?
        (( result == 0 )) && return 0
        (( result == 130 )) && return 130
        sleep 1
    done
    return 1
}

# Stop every process in a session politely (SIGINT = like Ctrl+C, lets nodes shut
# down cleanly - RTAB-Map writes the map file then), and force it (SIGKILL) if
# anything is still alive after 15 s.
stop_session() {
    local session_id=$1
    [[ -z "$session_id" ]] && return
    pkill -INT -s "$session_id" 2>/dev/null || return   # -s = match by session
    for _ in $(seq 15); do
        pgrep -s "$session_id" > /dev/null || return
        sleep 1
    done
    pkill -KILL -s "$session_id" 2>/dev/null
}

# Stops everything this run started (for the scripts' cleanup functions).
stop_everything() {
    echo
    echo "Stopping everything (takes up to ~15 s) ..."
    [[ -n "$WATCHER_ID" ]] && kill "$WATCHER_ID" 2>/dev/null
    stop_session "$SLAM_ID"
    stop_session "$SIM_ID"
    echo "Done. Logs are in $LOG_DIR"
}

# ---- Starting the parts -----------------------------------------------------------

# Starts the simulator and waits until the lidar and the odometry are running.
# Args: <rviz true|false> <step label, e.g. "[1/3]">.  Sets SIM_ID.
# static_map_to_odom:=false - the SLAM node publishes map->odom, so the sim must not.
start_simulator() {
    local sim_rviz=$1 step=$2
    echo "$step Starting the simulator, world '$WORLD' (log: $LOG_DIR/sim.log)"
    # $SIM_EXTRA_ARGS is deliberately NOT in quotes, so "a:=1 b:=2" becomes two arguments.
    SIM_ID=$(start_in_background "$LOG_DIR/sim.log" \
        ros2 launch unitree_go2_sim unitree_go2_launch.py static_map_to_odom:=false rviz:="$sim_rviz" \
        world:="$WORLD" $SIM_EXTRA_ARGS)

    echo "      waiting for the lidar ..."
    wait_for_message /velodyne_points/points 180 "$SIM_ID"
    case $? in
        0) ;;
        130) exit 130 ;;
        2) echo "      The simulator crashed. Last lines of its log:"; tail -15 "$LOG_DIR/sim.log"; exit 1 ;;
        *) echo "      No lidar data after 3 minutes. See $LOG_DIR/sim.log"; exit 1 ;;
    esac
    echo "      waiting for the robot's odometry ..."
    wait_for_message /odom 60 "$SIM_ID" || { echo "      No /odom - is CHAMP running? See $LOG_DIR/sim.log"; exit 1; }
    sleep 5   # give the robot a moment to finish standing up
}

# Starts navigation_rtabmap.launch.py (RTAB-Map + rviz [+ Nav2]) and waits until
# RTAB-Map works. Args: <step label> <description> <launch arguments ...>. Sets SLAM_ID.
# RTAB-Map sometimes starts but rejects every lidar scan and never produces a map
# (cause not found yet). It is easy to detect: a working RTAB-Map publishes /info about
# once per second, even while the robot stands still. If none arrives, the whole launch
# is restarted.
start_rtabmap() {
    local step=$1 description=$2; shift 2
    local attempt slam_log result
    for attempt in $(seq "$RTABMAP_ATTEMPTS"); do
        slam_log="$LOG_DIR/rtabmap_attempt$attempt.log"
        echo "$step Starting $description (log: $slam_log)"
        SLAM_ID=$(start_in_background "$slam_log" ros2 launch goto navigation_rtabmap.launch.py \
            database_path:="$MAP_FILE" delete_db:=false "$@")
        echo "      waiting for RTAB-Map to process its first scans ..."
        wait_for_message /info 30 "$SLAM_ID"
        result=$?
        (( result == 0 )) && return 0
        (( result == 130 )) && exit 130
        echo "      RTAB-Map is not producing a map (attempt $attempt/$RTABMAP_ATTEMPTS). Its last warnings:"
        grep -E "WARN|ERROR" "$slam_log" | grep rtabmap | tail -3 | cut -c1-200 | sed 's/^/        /'
        stop_session "$SLAM_ID"
        SLAM_ID=""
        if (( attempt == RTABMAP_ATTEMPTS )); then
            echo "      Giving up. Please send the warnings above / $slam_log"
            exit 1
        fi
        echo "      restarting it ..."
    done
}

# Run in the BACKGROUND ("&"): waits until RTAB-Map (in localization mode) has found the
# robot on the loaded map, then calls the given function once.
# Arg: <function to call when found>.
watch_localization() {
    local on_found=$1
    local started=$SECONDS hint_shown=false
    while kill -0 "$SLAM_ID" 2>/dev/null; do
        # RTAB-Map publishes /localization_pose (only in localization mode) every time it
        # matched the current scan against the loaded map.
        if timeout 5 ros2 topic echo /localization_pose --once --field header.stamp > /dev/null 2>&1; then
            "$on_found"
            return
        fi
        if [[ $hint_shown == false ]] && (( SECONDS - started > 30 )); then
            hint_shown=true
            echo
            echo ">>> Robot not found on the map yet. It is assumed to start where the map started."
            echo "    In rviz: click \"2D Pose Estimate\" and drag on the robot's real position/direction."
        fi
    done
}
