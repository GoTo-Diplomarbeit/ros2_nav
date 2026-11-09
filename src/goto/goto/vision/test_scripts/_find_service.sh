# Sourced by the test scripts: sets SERVICE to the container that runs the goto nodes.
# The containers can't exchange ROS messages with each other (Fast DDS uses shared
# memory, and /dev/shm is not shared), so commands must run in that same container.
# Set SERVICE yourself to skip the check.
if [[ -z "${SERVICE:-}" ]]; then
    for candidate in release-nvidia release; do
        if docker compose -f "$repo_root/docker-compose.yaml" exec -T "$candidate" \
            pgrep -f lib/goto/state_node >/dev/null 2>&1; then
            SERVICE="$candidate"
            break
        fi
    done
    if [[ -z "${SERVICE:-}" ]]; then
        echo "No container is running the goto nodes (state_node). Start bringup first." >&2
        exit 1
    fi
fi
