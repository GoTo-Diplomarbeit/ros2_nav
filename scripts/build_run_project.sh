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

# Matched against the processes inside the container. The [g] keeps the pattern from
# matching the pgrep command line itself.
launch_pattern='ros2 launch [g]oto bringup.launch.py'

# The container writes its process group here when the run starts ($$ makes the name
# unique per run), so cleanup stops exactly this tree and nothing else.
pgid_file="/tmp/goto_build_run_project.$$.pgid"

# Every "docker compose exec" is its own process group, so signalling the group also
# stops the nodes the launch spawned. Killing only ros2 launch leaves them running.
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

# Never start on top of leftovers. A process group whose leader is gone is the rest of
# a terminal that was killed - its nodes keep running and holding ports. A still
# running instance of this stack is stopped as well, so two stacks never compete.
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

        echo "Stopping the previous instance ..."
        for pgid in $pgids; do
            kill -INT -- "-$pgid" 2>/dev/null
        done
        for _ in $(seq 25); do
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
# set -e inside: if the build fails, the nodes must not start.
# exec: signals reach ros2 launch instead of an extra shell.
# <&0: background commands otherwise get /dev/null on stdin, and compose refuses a
# TTY-enabled exec whose stdin is not a terminal.
docker compose exec "${docker_exec[@]}" dev bash -c '
    echo "$(ps -o pgid= -p $$ | tr -d " ")" > "$1"
    set -e
    cd /ws
    source /opt/ros/jazzy/setup.bash
    source /unitree_go2_sim/install/setup.bash
    colcon build --symlink-install --packages-select goto
    source install/setup.bash
    exec ros2 launch goto bringup.launch.py log_level:=warn
' -- "$pgid_file" <&0 &
client_pid=$!

wait "$client_pid"
