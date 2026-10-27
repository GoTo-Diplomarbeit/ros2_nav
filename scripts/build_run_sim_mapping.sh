#!/usr/bin/env bash
# Starts the simulator, RTAB-Map navigation and teleop in one dev container session.
#
# Ctrl+C stops everything: SIGINT first, SIGKILL after a short grace period.
# Teleop needs a real terminal - an IDE run console has no TTY for the keys.
#
# Requires a built goto package; run scripts/build_run_project.sh first.
set -euo pipefail

cd "$(dirname "$0")/.."

# Reuse the running dev container, start it if needed.
docker compose up -d dev

# See build_run_project.sh: -T when there is no TTY (IDE run configurations).
docker_exec=()
if [ ! -t 0 ]; then
    docker_exec=(-T)
fi

# Matched against the processes inside the container. The [x] keeps the pattern from
# matching the pgrep command line itself.
launch_pattern='ros2 launch ([u]nitree_go2_sim unitree_go2_launch.py|[g]oto navigation_rtabmap.launch.py)|[t]eleop_twist_keyboard|[g]z sim'

# The container writes its process group here when the run starts ($$ makes the name
# unique per run), so cleanup stops exactly this tree and nothing else.
pgid_file="/tmp/goto_build_run_sim_mapping.$$.pgid"

# Every "docker compose exec" is its own process group, so signalling the group also
# stops the nodes the launches spawned (Gazebo included).
stop_current() {
    docker compose exec -T dev bash -c '
        pgid=$(cat "$1" 2>/dev/null) || exit 0
        rm -f "$1"
        [ -n "$pgid" ] || exit 0
        kill -INT -- "-$pgid" 2>/dev/null
        for _ in $(seq 25); do
            pgrep -g "$pgid" >/dev/null 2>&1 || break
            sleep 0.2
        done
        kill -KILL -- "-$pgid" 2>/dev/null
        exit 0
    ' -- "$pgid_file" || true
}

# Never start on top of leftovers: two simulators publish two /clock streams, and every
# node then logs "Detected jump back in time" and clears its TF buffer. Leftovers are
# process groups without a leader (the terminal that started them was killed); a still
# running stack (leader alive) is stopped as well.
preflight() {
    docker compose exec -T dev bash -c '
        self=$(ps -o pgid= -p $$ | tr -d " ")

        for pgid in $(ps -eo pgid= | tr -d " " | sort -u); do
            [ "$pgid" = "1" ] && continue
            [ "$pgid" = "$self" ] && continue
            kill -0 "$pgid" 2>/dev/null && continue
            kill -KILL -- "-$pgid" 2>/dev/null
        done

        pgids=$(pgrep -f "$1" 2>/dev/null | while read -r pid; do
            ps -o pgid= -p "$pid" 2>/dev/null | tr -d " "
        done | sort -u | grep -vx 1 | grep -vx "$self")
        [ -n "$pgids" ] || exit 0

        echo "Stopping the already running stack ..."
        for pgid in $pgids; do
            kill -INT -- "-$pgid" 2>/dev/null
        done
        for _ in $(seq 50); do
            pgrep -f "$1" >/dev/null 2>&1 || break
            sleep 0.2
        done
        for pgid in $pgids; do
            kill -KILL -- "-$pgid" 2>/dev/null
        done
        exit 0
    ' -- "$1" || true
}

cleanup() {
    if [ -n "${client_pid:-}" ]; then
        kill -INT "$client_pid" 2>/dev/null || true
    fi
    stop_current
}
trap cleanup INT TERM EXIT

preflight "$launch_pattern"

# $1: file in which the container records its process group.
# <&0: background commands otherwise get /dev/null on stdin, and compose refuses a
# TTY-enabled exec whose stdin is not a terminal.
docker compose exec "${docker_exec[@]}" dev bash -c '
    echo "$(ps -o pgid= -p $$ | tr -d " ")" > "$1"
    set -e
    cd /ws
    source /opt/ros/jazzy/setup.bash
    source /unitree_go2_sim/install/setup.bash

    if [ ! -f /ws/install/setup.bash ]; then
        echo "goto is not built yet - run scripts/build_run_project.sh first" >&2
        exit 1
    fi
    source /ws/install/setup.bash

    ros2 launch unitree_go2_sim unitree_go2_launch.py static_map_to_odom:=false rviz:=false &
    sleep 30

    ros2 launch goto navigation_rtabmap.launch.py &
    sleep 10

    # Teleop runs in the foreground so it receives your keys.
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
' -- "$pgid_file" <&0 &
client_pid=$!

wait "$client_pid"
