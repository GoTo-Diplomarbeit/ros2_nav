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

# Matched against the processes inside the container when stopping. The [x] keeps the
# pattern from matching the pkill command line itself.
launch_pattern='ros2 launch ([u]nitree_go2_sim unitree_go2_launch.py|[g]oto navigation_rtabmap.launch.py)|[t]eleop_twist_keyboard|[g]z sim'

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

docker compose exec "${docker_exec[@]}" dev bash -c '
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
' &
client_pid=$!

wait "$client_pid"
