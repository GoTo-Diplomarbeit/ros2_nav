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

# Matched against the processes inside the container when stopping. The [u] keeps the
# pattern from matching the pkill command line itself.
launch_pattern='ros2 launch [u]nitree_go2_sim unitree_go2_launch.py|[g]z sim'

cleanup() {
    if [ -n "${client_pid:-}" ]; then
        kill -INT "$client_pid" 2>/dev/null || true
    fi
    # Without a TTY the signal is not forwarded into the container.
    if docker compose exec -T dev pkill -INT -f "$launch_pattern" >/dev/null 2>&1; then
        for _ in $(seq 50); do
            docker compose exec -T dev pgrep -f "$launch_pattern" >/dev/null 2>&1 || break
            sleep 0.2
        done
        docker compose exec -T dev pkill -KILL -f "$launch_pattern" >/dev/null 2>&1 || true
    fi
}
trap cleanup INT TERM EXIT

# The arguments after -- become "$@" of the shell inside the container.
docker compose exec "${docker_exec[@]}" dev bash -c '
    cd /ws
    source /opt/ros/jazzy/setup.bash
    source /unitree_go2_sim/install/setup.bash
    exec ros2 launch unitree_go2_sim unitree_go2_launch.py "$@"
' -- "${launch_args[@]}" &
client_pid=$!

wait "$client_pid"
