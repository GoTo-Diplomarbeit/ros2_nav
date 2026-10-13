#!/usr/bin/env bash
# =============================================================================
# run_sim_mapping.sh - start EVERYTHING for mapping from ONE terminal
#
#   1. Gazebo simulator with the Go2 (CHAMP controller, sensors, EKFs)
#   2. SLAM + rviz + Nav2   (RTAB-Map by default, or slam_toolbox)
#   3. Keyboard teleop, right here in this terminal
#
# Usage:
#   scripts/run_sim_mapping.sh                  # RTAB-Map: 3D map + 2D map
#   scripts/run_sim_mapping.sh slam_toolbox     # the old 2D-only pipeline
#
# Stop EVERYTHING: press Ctrl+C in this terminal (while teleop is running).
#
# The simulator and SLAM print a lot, so their output goes to log files instead
# of this terminal (the script prints where). Watch one live with: tail -f <file>
#
# Pitfalls:
#   - Only one copy may run at a time. Two simulators or two SLAM nodes both
#     publish /map and map->odom and the map breaks. The script refuses to start
#     if it finds one already running.
#   - The teleop only receives keys while THIS terminal window has focus.
#   - RTAB-Map's map is saved in ~/.ros/rtabmap.db, and the NEXT start deletes it.
#     Copy it away first to keep it:  cp ~/.ros/rtabmap.db ~/maps/<name>.db
#   - Don't steer with the keyboard while Nav2 is driving to a goal: both send
#     /cmd_vel and the robot stutters between the two.
#   - New launch/config files still need a build first:
#       colcon build --symlink-install --packages-select go2_nav
# =============================================================================

# ---- Settings you may want to change ----------------------------------------
# Slow, smooth driving gives the cleanest map; fast spins on the spot are where the
# odometry drifts. CHAMP caps the robot at 0.3 m/s and 0.5 rad/s anyway
# (max_linear_velocity_x / max_angular_velocity_z in the sim's config/gait/gait.yaml),
# so higher values here would not make it faster.
TELEOP_SPEED=0.3      # m/s
TELEOP_TURN=0.5       # rad/s
SIM_EXTRA_ARGS=""     # extra simulator arguments, separated by spaces, e.g.
                      # "disable_d455:=true" saves CPU (SLAM doesn't use the camera)
RTABMAP_ATTEMPTS=3    # how often to restart RTAB-Map if it gets stuck at startup
# -----------------------------------------------------------------------------

SLAM="${1:-rtabmap}"
if [[ "$SLAM" != "rtabmap" && "$SLAM" != "slam_toolbox" ]]; then
    echo "Unknown SLAM backend '$SLAM'. Use: rtabmap (default) or slam_toolbox"
    exit 1
fi

# The repo root, found relative to this script, so it works wherever the repo is.
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOG_DIR="$HOME/.ros/log/go2_run_$(date +%Y-%m-%d_%H-%M-%S)"
mkdir -p "$LOG_DIR"

# ---- 0. Load the three ROS environments --------------------------------------
# Order matters: ROS itself, then the Go2 sim workspace, then this repo on top.
for setup_file in /opt/ros/jazzy/setup.bash "$HOME/go2_ws/install/setup.bash" "$REPO_DIR/install/setup.bash"; do
    if [[ ! -f "$setup_file" ]]; then
        echo "Missing $setup_file - has that workspace been built?"
        exit 1
    fi
    source "$setup_file"
done

# ---- Refuse to start a second copy -------------------------------------------
# The patterns are anchored to the START of each command line ("^"), so they only
# match the real programs - not e.g. a terminal or editor whose command line just
# happens to contain the words "gz sim".
already_running=$(pgrep -af "^gz sim|^[^ ]*rtabmap_slam/rtabmap|^[^ ]*slam_toolbox/[^ ]*slam_toolbox_node|^[^ ]*nav2_bt_navigator/bt_navigator|^[^ ]*python3 [^ ]*/teleop_twist_keyboard( |$)")
if [[ -n "$already_running" ]]; then
    echo "Something from a previous run is still running:"
    echo "$already_running" | sed 's/^/    /' | cut -c1-120
    echo "Stop it first (Ctrl+C in its terminal), otherwise two copies fight over the map."
    exit 1
fi

# ---- Helpers ------------------------------------------------------------------
SIM_ID=""
SLAM_ID=""

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
# down cleanly), and force it (SIGKILL) if anything is still alive after 15 s.
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

cleanup() {
    trap - EXIT INT TERM HUP
    echo
    echo "Stopping everything (takes up to ~15 s) ..."
    stop_session "$SLAM_ID"
    stop_session "$SIM_ID"
    echo "Done. Logs are in $LOG_DIR"
    if [[ "$SLAM" == "rtabmap" && -n "$SLAM_ID" ]]; then
        echo "Your map is in ~/.ros/rtabmap.db - the next start DELETES it. To keep it:"
        echo "    mkdir -p ~/maps && cp ~/.ros/rtabmap.db ~/maps/map_$(date +%Y-%m-%d_%H-%M).db"
    fi
}
trap cleanup EXIT             # runs however the script ends
trap 'exit 130' INT TERM HUP  # Ctrl+C / closed window -> exit -> cleanup

# ---- 1. Simulator --------------------------------------------------------------
# static_map_to_odom:=false - the SLAM node publishes map->odom, so the sim must not.
# rviz: RTAB-Map's launch brings its own rviz (3D view); slam_toolbox uses the sim's.
# $SIM_EXTRA_ARGS is deliberately NOT in quotes, so "a:=1 b:=2" becomes two arguments.
if [[ "$SLAM" == "rtabmap" ]]; then sim_rviz=false; else sim_rviz=true; fi
echo "[1/3] Starting the simulator          (log: $LOG_DIR/sim.log)"
SIM_ID=$(start_in_background "$LOG_DIR/sim.log" \
    ros2 launch unitree_go2_sim unitree_go2_launch.py static_map_to_odom:=false rviz:=$sim_rviz $SIM_EXTRA_ARGS)

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

# ---- 2. SLAM + rviz + Nav2 ------------------------------------------------------
if [[ "$SLAM" == "rtabmap" ]]; then
    # RTAB-Map sometimes starts but rejects every lidar scan and never produces a map
    # (cause not found yet). It is easy to detect: a working RTAB-Map publishes /info
    # about once per second, even while the robot stands still. If none arrives, the
    # whole SLAM launch is restarted.
    for attempt in $(seq "$RTABMAP_ATTEMPTS"); do
        slam_log="$LOG_DIR/rtabmap_attempt$attempt.log"
        echo "[2/3] Starting RTAB-Map + rviz + Nav2 (log: $slam_log)"
        SLAM_ID=$(start_in_background "$slam_log" ros2 launch go2_nav navigation_rtabmap.launch.py)
        echo "      waiting for RTAB-Map to process its first scans ..."
        wait_for_message /info 30 "$SLAM_ID"
        result=$?
        (( result == 0 )) && break
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
else
    slam_log="$LOG_DIR/slam_toolbox.log"
    echo "[2/3] Starting slam_toolbox + Nav2     (log: $slam_log)"
    SLAM_ID=$(start_in_background "$slam_log" ros2 launch go2_nav navigation.launch.py)
    echo "      waiting for the first map ..."
    wait_for_message /map 60 "$SLAM_ID" || { echo "      No /map after 60 s. See $slam_log"; exit 1; }
fi
echo "      mapping is running."

# ---- 3. Teleop (foreground) -----------------------------------------------------
# Runs in THIS terminal so it receives your key presses. When you press Ctrl+C,
# teleop ends, the script reaches its end, and cleanup stops everything else.
echo "[3/3] Teleop - keep this window focused. Ctrl+C here stops EVERYTHING."
echo
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
    -p speed:=$TELEOP_SPEED -p turn:=$TELEOP_TURN
