#!/usr/bin/env bash
# Builds the goto package and starts the goto nodes, mapping and navigation.
#
# Runs on the host (terminal or PyCharm "Shell Script" run configuration) and drives
# Docker itself - no Docker Compose interpreter needed.
set -euo pipefail

cd "$(dirname "$0")/.."

# Reuse the running dev container, start it if needed.
docker compose up -d dev

# With a TTY, Ctrl+C reaches the nodes. Run configurations have no TTY, and without
# -T docker refuses to start the exec session.
docker_exec=()
if [ ! -t 0 ]; then
    docker_exec=(-T)
fi

# Matched against the processes inside the container when stopping. The [g] keeps the
# pattern from matching the pkill command line itself.
launch_pattern='ros2 launch [g]oto bringup.launch.py'

cleanup() {
    if [ -n "${client_pid:-}" ]; then
        kill -INT "$client_pid" 2>/dev/null || true
    fi
    # Without a TTY the signal is not forwarded into the container. Leftover nodes
    # would keep the web server's port 8000 busy for the next start.
    if docker compose exec -T dev pkill -INT -f "$launch_pattern" >/dev/null 2>&1; then
        for _ in $(seq 25); do
            docker compose exec -T dev pgrep -f "$launch_pattern" >/dev/null 2>&1 || break
            sleep 0.2
        done
        docker compose exec -T dev pkill -KILL -f "$launch_pattern" >/dev/null 2>&1 || true
    fi
}
trap cleanup INT TERM EXIT

# set -e inside: if the build fails, the nodes must not start.
# exec: signals reach ros2 launch instead of an extra shell.
docker compose exec "${docker_exec[@]}" dev bash -c '
    set -e
    cd /ws
    source /opt/ros/jazzy/setup.bash
    source /unitree_go2_sim/install/setup.bash
    colcon build --symlink-install --packages-select goto
    source install/setup.bash
    exec ros2 launch goto bringup.launch.py log_level:=warn
' &
client_pid=$!

wait "$client_pid"
