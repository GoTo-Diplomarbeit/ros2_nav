#!/usr/bin/env bash
# Starts the Go2 simulator (Gazebo) in the dev container.
#
# Usage:
#   scripts/launch_gazebo.sh                          # lidar + RViz
#   scripts/launch_gazebo.sh --disable_lidar true     # no lidar, no D455 camera
#   scripts/launch_gazebo.sh --disable_rviz           # no RViz
#   scripts/launch_gazebo.sh world:=school_with_person
#
# Any remaining "name:=value" argument is passed on to ros2 launch (e.g. world:=).
set -euo pipefail

cd "$(dirname "$0")/.."

disable_lidar=false
disable_rviz=false

usage() {
    echo "Usage: $(basename "$0") [--disable_lidar [true|false]] [--disable_rviz [true|false]] [name:=value ...]"
}

# A flag works with or without an explicit value, so the old calls
# ("--disable_lidar true") keep working. Sets flag_value and flag_args (1 or 2).
parse_flag() {
    flag_value=true
    flag_args=1
    case "${2:-}" in
        true|True|TRUE)      flag_value=true;  flag_args=2 ;;
        false|False|FALSE)   flag_value=false; flag_args=2 ;;
    esac
}

launch_args=()
while [ "$#" -gt 0 ]; do
    case "$1" in
        --disable_lidar)
            parse_flag "$@"
            disable_lidar=$flag_value
            shift "$flag_args"
            ;;
        --disable_rviz)
            parse_flag "$@"
            disable_rviz=$flag_value
            shift "$flag_args"
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *:=*)
            launch_args+=("$1")
            shift
            ;;
        *)
            echo "Unknown argument: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [ "$disable_lidar" = true ]; then
    launch_args+=(disable_velodyne_lidar:=true disable_d455:=true)
fi
if [ "$disable_rviz" = true ]; then
    launch_args+=(rviz:=false)
fi

# Reuse the running dev container, start it if needed.
docker compose up -d dev

# See build_run_project.sh: -T when there is no TTY (IDE run configurations).
docker_exec=()
if [ ! -t 0 ]; then
    docker_exec=(-T)
fi

# Matched against the processes inside the container. The [u] / [g] keep the pattern
# from matching the pgrep command line itself.
launch_pattern='ros2 launch [u]nitree_go2_sim unitree_go2_launch.py|[g]z sim'

# The container writes its process group here when the run starts ($$ makes the name
# unique per run), so cleanup stops exactly this tree and nothing else.
pgid_file="/tmp/goto_launch_gazebo.$$.pgid"

# Every "docker compose exec" is its own process group, so signalling the group also
# stops the nodes the launch spawned (Gazebo included).
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

# Never start on top of leftovers: two simulators publish two /clock streams, and
# every node then logs "Detected jump back in time" and clears its TF buffer. Leftovers
# are process groups without a leader (the terminal that started them was killed);
# a still running simulator (leader alive) is stopped as well.
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

        echo "Stopping the previous simulator ..."
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

# $1: file in which the container records its process group, then shifts away.
# <&0: background commands otherwise get /dev/null on stdin, and compose refuses a
# TTY-enabled exec whose stdin is not a terminal.
docker compose exec "${docker_exec[@]}" dev bash -c '
    echo "$(ps -o pgid= -p $$ | tr -d " ")" > "$1"
    shift
    cd /ws
    source /opt/ros/jazzy/setup.bash
    source /unitree_go2_sim/install/setup.bash
    exec ros2 launch unitree_go2_sim unitree_go2_launch.py "$@"
' -- "$pgid_file" "${launch_args[@]}" <&0 &
client_pid=$!

wait "$client_pid"
