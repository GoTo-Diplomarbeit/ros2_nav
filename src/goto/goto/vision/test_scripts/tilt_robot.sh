#!/usr/bin/env bash
# Tilt the robot body by publishing an orientation to /body_pose.
# The CHAMP controller applies /body_pose instantly, so a single big step makes the feet slip and the
# robot slide sideways. This script ramps from the last tilt to the new one in small steps instead.
#
# Usage: tilt_robot.sh [y] [duration]
#   y         quaternion y component, default -0.1, must be between -1 and 1 (0 = level)
#   duration  seconds the tilt takes, default 2.0 (bigger = slower)
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../../.." && pwd)"
y="${1:--0.1}"
duration="${2:-2.0}"

number='^-?[0-9]*\.?[0-9]+$'
if ! [[ "$y" =~ $number ]] || ! awk -v y="$y" 'BEGIN { exit !(y >= -1 && y <= 1) }'; then
    echo "Invalid y value: '$y' (expected a number between -1 and 1, e.g. -0.1)" >&2
    exit 1
fi
if ! [[ "$duration" =~ $number ]] || ! awk -v d="$duration" 'BEGIN { exit !(d > 0) }'; then
    echo "Invalid duration: '$duration' (expected seconds > 0, e.g. 2.0)" >&2
    exit 1
fi

source "$(dirname -- "${BASH_SOURCE[0]}")/_find_service.sh"

docker compose -f "$repo_root/docker-compose.yaml" exec -T "$SERVICE" bash -c '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /unitree_go2_sim/install/setup.bash
    cd /ws
    source install/setup.bash
    exec python3 - "$@"
' bash "$y" "$duration" <<'PY'
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Pose

RATE_HZ = 50
# last commanded y, so the next call ramps from there (lives in the container, reset when it restarts)
STATE_FILE = "/tmp/tilt_robot_last_y"

target_y = float(sys.argv[1])
duration = float(sys.argv[2])

try:
    with open(STATE_FILE) as f:
        start_y = float(f.read())
except (OSError, ValueError):
    start_y = 0.0  # assume the robot is level

# interpolate the pitch angle (not the raw quaternion component) so the motion is even
start_pitch = 2 * math.asin(start_y)
target_pitch = 2 * math.asin(target_y)

rclpy.init()
node = rclpy.create_node("tilt_robot")
pub = node.create_publisher(Pose, "/body_pose", 1)

# wait for the controller to be connected, otherwise the first messages are lost
deadline = time.monotonic() + 5.0
while pub.get_subscription_count() == 0:
    if time.monotonic() > deadline:
        print("No subscriber on /body_pose (is the robot controller running?)", file=sys.stderr)
        sys.exit(1)
    time.sleep(0.05)

print(f"Tilting y: {start_y:.3f} -> {target_y:.3f} over {duration:.1f} s")

steps = max(1, int(duration * RATE_HZ))
for i in range(1, steps + 1):
    # smooth start and stop (cosine ease-in-out) to avoid jerks at both ends
    t = 0.5 - 0.5 * math.cos(math.pi * i / steps)
    pitch = start_pitch + (target_pitch - start_pitch) * t
    msg = Pose()
    msg.orientation.y = math.sin(pitch / 2)
    msg.orientation.w = math.cos(pitch / 2)
    pub.publish(msg)
    time.sleep(1.0 / RATE_HZ)

with open(STATE_FILE, "w") as f:
    f.write(str(target_y))

node.destroy_node()
rclpy.shutdown()
PY
