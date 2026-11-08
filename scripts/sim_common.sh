#!/usr/bin/env bash
# Shared code of build_run_sim_mapping.sh and build_run_sim_navigation.sh (loaded with "source").
#
# Everything runs inside the Docker container. Started on the host, a script starts the
# container and then runs itself again inside it (run_in_docker). Only the project folder
# is shared with the host (it is /ws in the container), so maps and logs are kept there:
#   maps/<world>.db        one RTAB-Map map per world (gitignored)
#   log/sim_runs/<date>/   the logs of each run
#
# No "set -e" on purpose: the wait functions return error codes that the scripts check.

# Sensors the sim doesn't need to render for mapping/navigation (saves a lot of CPU): only the
# Velodyne, IMU and leg data are used. vision_node/camera_node get no images in these runs.
SIM_EXTRA_ARGS="disable_d455:=true disable_camera:=true disable_lidar_l1:=true"
RTABMAP_ATTEMPTS=3              # restarts if RTAB-Map gets stuck at startup
RTABMAP_STOP_SECONDS=60         # time RTAB-Map gets to save the map (a running map update took 16 s)
SERVICE="${SERVICE:-release}"   # Docker service; "release-nvidia" renders on the NVIDIA GPU
WORLDS_DIR=/unitree_go2_sim/install/unitree_go2_description/share/unitree_go2_description/worlds

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # the project folder (/ws in the container)
MAP_DIR="$REPO_DIR/maps"

# Ids of what this run started, so the cleanup stops exactly these
SIM_ID=""
RTABMAP_ID=""
WATCHER_ID=""

# Programs of which only one copy may run - two would fight over /map and map->odom.
# "^" = start of the command line, so e.g. an editor that has "gz sim" in its title doesn't match.
RUNNING_PATTERNS=(
    "^gz sim"
    "^[^ ]*rtabmap_slam/rtabmap"
    "^[^ ]*slam_toolbox/[^ ]*slam_toolbox_node"
    "^[^ ]*nav2_bt_navigator/bt_navigator"
    "^[^ ]*python3 [^ ]*/teleop_twist_keyboard( |$)"
)

# ---- Docker ---------------------------------------------------------------------

# Exits if a sim, SLAM, Nav2 or teleop is still running. On the host this also sees the
# processes inside every container (release and release-nvidia).
refuse_if_already_running() {
    local running
    running=$(pgrep -af "$(IFS='|'; echo "${RUNNING_PATTERNS[*]}")")
    [[ -z "$running" ]] && return 0
    echo "Still running from an earlier run:"
    echo "$running" | cut -c1-120 | sed 's/^/    /'
    echo "Stop it with Ctrl+C in its terminal. If that terminal is gone, stop it like this (the map is saved):"
    echo "    sudo kill -INT -- -\$(pgrep -of '^bash /ws/scripts/build_run_sim')"
    exit 1
}

# Sends Ctrl+C to the run script inside the container - to its whole process group, as a
# terminal does, so teleop ends too. The script then stops everything and RTAB-Map saves the map.
# No input/output: this runs when the terminal is already gone.
stop_script_in_container() {
    docker compose exec -T "$SERVICE" \
        bash -c 'kill -INT -- -"$(pgrep -of "^bash /ws/scripts/build_run_sim")"' < /dev/null > /dev/null 2>&1
}

# On the host: starts the container and runs the calling script inside it, with the same
# arguments. Inside the container (/.dockerenv exists there) it returns at once.
run_in_docker() {
    [[ -f /.dockerenv ]] && return 0
    refuse_if_already_running
    cd "$REPO_DIR" || exit 1
    docker compose up -d --remove-orphans "$SERVICE" || exit 1

    # Closing this terminal doesn't stop the script inside the container (Docker doesn't pass
    # that on), so do it ourselves
    trap stop_script_in_container HUP TERM

    # TZ: the container runs on UTC; ROS_DOMAIN_ID: keeps two sims on one network apart
    docker compose exec \
        -e TZ="$(cat /etc/timezone 2>/dev/null)" \
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
        "$SERVICE" bash "/ws/scripts/$(basename "$0")" "$@"
    exit $?
}

# The container runs as root, so the files it creates would belong to root on the host.
# Give them the owner of the project folder. (With rootless Docker the folder already shows
# as root here and nothing has to change.)
give_files_back() {
    local owner
    owner=$(stat -c %u:%g "$REPO_DIR")
    [[ "$owner" == "0:0" ]] && return 0
    chown -R "$owner" "$MAP_DIR" "$REPO_DIR/log" "$REPO_DIR/build" "$REPO_DIR/install" 2>/dev/null
    return 0
}

# Builds goto, so the run always uses the current code (about 2 s).
build_goto() {
    # A build made outside Docker links to host paths that don't exist in the container
    if [[ -n "$(find "$REPO_DIR/install/goto" -xtype l -print -quit 2>/dev/null)" ]]; then
        echo "build/ and install/ are from a build outside Docker. Delete them once: rm -rf build install log"
        exit 1
    fi
    echo "Building goto ..."
    if ! (cd "$REPO_DIR" && colcon build --symlink-install --packages-select goto) > "$LOG_DIR/build.log" 2>&1; then
        tail -20 "$LOG_DIR/build.log"
        echo "The build failed (log: $LOG_DIR/build.log)"
        exit 1
    fi
}

# Checks $WORLD, loads ROS, builds goto, creates the log folder and works out the map file.
# Also sets the traps: Ctrl+C or a closed terminal -> exit -> stop_on_exit.
# Sets LOG_DIR, MAP_NAME, MAP_FILE.
setup_run() {
    trap stop_on_exit EXIT
    trap 'exit 130' INT TERM HUP
    refuse_if_already_running   # again, in case the script was started inside the container

    # Catch a wrong world name now, not after waiting minutes for the simulator
    if [[ ! -f "$WORLD" && ! -f "$WORLDS_DIR/$WORLD.sdf" ]]; then
        echo "Unknown world '$WORLD'. Available: $(ls "$WORLDS_DIR" | sed 's/\.sdf$//' | tr '\n' ' ')"
        exit 1
    fi

    LOG_DIR="$REPO_DIR/log/sim_runs/$(date +%Y-%m-%d_%H-%M-%S)"
    mkdir -p "$LOG_DIR"
    MAP_NAME="$(basename "$WORLD" .sdf)"   # "default" -> default, "/ws/x/my_world.sdf" -> my_world
    MAP_FILE="$MAP_DIR/$MAP_NAME.db"

    source /opt/ros/jazzy/setup.bash
    source /unitree_go2_sim/install/setup.bash   # the simulator, built into the Docker image
    build_goto
    source "$REPO_DIR/install/setup.bash"
}

# ---- Starting and stopping ----------------------------------------------------------

# Starts a command in the background, in its own session, with its output in a log file.
# Prints its id. Why a session: everything the launch starts stays in it (even Gazebo's
# server and GUI), so stop_session can stop all of it and nothing is left over.
start_in_background() {
    local log_file=$1; shift
    setsid "$@" < /dev/null > "$log_file" 2>&1 &
    local id=$!
    # "&" returns a moment before setsid has created the session - wait until it exists
    for _ in $(seq 50); do
        [[ "$(ps -o sid= -p "$id" 2>/dev/null | tr -d ' ')" == "$id" ]] && break
        sleep 0.1
    done
    echo "$id"
}

# Waits until a message arrives on a topic. Returns 0 = got one, 1 = timed out,
# 2 = the launch with the given id has died.
wait_for_message() {
    local topic=$1 max_seconds=$2 launch_id=$3
    local deadline=$((SECONDS + max_seconds))
    while (( SECONDS < deadline )); do
        kill -0 "$launch_id" 2>/dev/null || return 2   # "kill -0" only checks that it's alive
        # Fails at once while the topic doesn't exist yet, hence the loop
        timeout 5 ros2 topic echo "$topic" --once --field header.stamp > /dev/null 2>&1 && return 0
        sleep 1
    done
    return 1
}

# Stops every process of a session: first like Ctrl+C (RTAB-Map saves the map then),
# by force after <max_seconds> (default 15). Returns 1 if it had to use force.
stop_session() {
    local session_id=$1 max_seconds=${2:-15}
    [[ -z "$session_id" ]] && return 0
    pkill -INT -s "$session_id" 2>/dev/null || return 0
    for _ in $(seq "$max_seconds"); do
        pgrep -s "$session_id" > /dev/null || return 0
        sleep 1
    done
    pkill -KILL -s "$session_id" 2>/dev/null
    return 1
}

# Stops everything this run started and says if the map may not have been saved.
# Returns 0 = map saved (or RTAB-Map never started), 1 = RTAB-Map had to be killed,
# 2 = RTAB-Map had already died during the run.
stop_everything() {
    [[ -n "$WATCHER_ID" ]] && kill "$WATCHER_ID" 2>/dev/null
    if [[ -z "$SIM_ID" ]]; then   # failed before anything was started
        give_files_back
        return 0
    fi
    local result=0
    echo
    echo "Stopping everything (saving the map can take up to a minute) ..."
    if [[ -n "$RTABMAP_ID" ]] && ! kill -0 "$RTABMAP_ID" 2>/dev/null; then
        result=2
        echo "RTAB-Map had stopped by itself during the run - the map lacks what came after (see its log)."
    elif ! stop_session "$RTABMAP_ID" "$RTABMAP_STOP_SECONDS"; then
        result=1
        echo "RTAB-Map had to be killed - the map may be incomplete."
    fi
    stop_session "$SIM_ID"
    give_files_back
    echo "Done. Logs: $LOG_DIR"
    return $result
}

# Messages a script wants to print after everything stopped. It gets stop_everything's result.
# The mapping script defines its own; this one does nothing.
after_stop() { :; }

# Runs whenever a run script ends (see setup_run). Ignores a second Ctrl+C, so the shutdown
# can always finish.
stop_on_exit() {
    trap '' INT TERM HUP
    stop_everything
    after_stop $?
}

# Starts the simulator and waits until the lidar and the odometry run. Sets SIM_ID.
# static_map_to_odom:=false - RTAB-Map publishes map->odom, so the sim must not.
# rviz:=false - navigation_rtabmap.launch.py brings its own rviz.
start_simulator() {
    echo "[1/3] Starting the simulator, world '$WORLD' (log: $LOG_DIR/sim.log)"
    # $SIM_EXTRA_ARGS without quotes on purpose, so "a:=1 b:=2" becomes two arguments
    SIM_ID=$(start_in_background "$LOG_DIR/sim.log" \
        ros2 launch unitree_go2_sim unitree_go2_launch.py static_map_to_odom:=false rviz:=false \
        world:="$WORLD" $SIM_EXTRA_ARGS)

    echo "      waiting for the lidar ..."
    wait_for_message /velodyne_points/points 180 "$SIM_ID"
    case $? in
        0) ;;
        2) echo "      The simulator crashed:"; tail -15 "$LOG_DIR/sim.log"; exit 1 ;;
        *) echo "      No lidar data after 3 minutes. See $LOG_DIR/sim.log"; exit 1 ;;
    esac
    echo "      waiting for the robot's odometry ..."
    wait_for_message /odom 60 "$SIM_ID" || { echo "      No /odom - is CHAMP running? See $LOG_DIR/sim.log"; exit 1; }
    sleep 5   # give the robot a moment to finish standing up
}

# Starts navigation_rtabmap.launch.py with this world's map and waits until RTAB-Map works.
# Args: <description> <launch arguments ...>. Sets RTABMAP_ID.
# RTAB-Map sometimes starts but processes no scans (cause not found yet). A working one
# publishes /info about once per second, so if none arrives, the launch is restarted.
start_rtabmap() {
    local description=$1; shift
    local attempt log_file
    for attempt in $(seq "$RTABMAP_ATTEMPTS"); do
        log_file="$LOG_DIR/rtabmap_attempt$attempt.log"
        echo "[2/3] Starting $description (log: $log_file)"
        RTABMAP_ID=$(start_in_background "$log_file" ros2 launch goto navigation_rtabmap.launch.py \
            database_path:="$MAP_FILE" delete_db:=false "$@")
        echo "      waiting for RTAB-Map to process its first scans ..."
        wait_for_message /info 30 "$RTABMAP_ID" && return 0

        echo "      RTAB-Map processes no scans (attempt $attempt/$RTABMAP_ATTEMPTS). Its last warnings:"
        grep -E "WARN|ERROR" "$log_file" | grep rtabmap | tail -3 | cut -c1-200 | sed 's/^/        /'
        stop_session "$RTABMAP_ID" "$RTABMAP_STOP_SECONDS"   # it saves the loaded map when it stops
        RTABMAP_ID=""
    done
    echo "      Giving up. See $LOG_DIR"
    exit 1
}

# Run in the background ("&"): waits until RTAB-Map (localization mode) has found the robot
# on the loaded map, then calls the given function once.
watch_localization() {
    local on_found=$1 started=$SECONDS hint_shown=false
    while kill -0 "$RTABMAP_ID" 2>/dev/null; do
        # /localization_pose comes each time a scan matched the loaded map
        if timeout 5 ros2 topic echo /localization_pose --once --field header.stamp > /dev/null 2>&1; then
            "$on_found"
            return
        fi
        if [[ $hint_shown == false ]] && (( SECONDS - started > 30 )); then
            hint_shown=true
            echo ">>> Robot not found on the map yet. It is assumed to start where the map started."
            echo "    If it doesn't, click \"2D Pose Estimate\" in rviz on its real position."
        fi
    done
}
