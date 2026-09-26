#!/bin/bash
# Set up the ROS 2 + Go2 environment, then run a python script from this repo.
#
#   ./run.sh                              runs movement/MovementNode.py
#   ./run.sh movement/other.py    runs any other script in the repo
#   ./run.sh other.py extra args          ...passing extra arguments through
#   source ./run.sh                        sets the environment up in YOUR shell
#                                          and runs nothing (then use ros2 topic
#                                          echo, python3 ..., etc.)
#
# Start the simulator first, in another terminal:
#   ros2 launch unitree_go2_sim unitree_go2_launch.py
# and wait ~30 s for the controllers to become active. Ctrl-C stops the script.
#
# Overridable if your layout differs:
#   ROS_SETUP, WS_SETUP, PYTHON

set -e

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
WS_SETUP="${WS_SETUP:-$HOME/go2_ws/install/setup.bash}"
PYTHON_BIN="${PYTHON:-python3}"
DEFAULT_SCRIPT="$REPO/unitree_movement/walk_forward.py"

# --- checks ------------------------------------------------------------------
if [ ! -r "$ROS_SETUP" ]; then
  echo "ERROR: $ROS_SETUP not found." >&2
  echo "       Is ROS 2 Jazzy installed?  Override with ROS_SETUP=/path/to/setup.bash" >&2
  exit 1
fi
if [ ! -r "$WS_SETUP" ]; then
  echo "ERROR: $WS_SETUP not found - the Go2 workspace is not built." >&2
  echo "       Build it first:  cd ~/go2_ws && colcon build" >&2
  echo "       (or point WS_SETUP at a different overlay)" >&2
  exit 1
fi
command -v "$PYTHON_BIN" >/dev/null 2>&1 || {
  echo "ERROR: '$PYTHON_BIN' not found on PATH. Override with PYTHON=/path/to/python3" >&2
  exit 1
}

# shellcheck disable=SC1090
source "$ROS_SETUP"
# shellcheck disable=SC1090
source "$WS_SETUP"

# --- sourced instead of executed: set the environment up and stop -------------
if [ "${BASH_SOURCE[0]}" != "$0" ]; then
  echo "ROS 2 Jazzy + Go2 overlay are sourced in this shell (PYTHONPATH, LD_LIBRARY_PATH, AMENT_PREFIX_PATH)."
  return 0 2>/dev/null || true
fi

# --- pick the script ---------------------------------------------------------
case "${1:-}" in
  -h|--help)
    sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'
    exit 0
    ;;
esac

TARGET="$DEFAULT_SCRIPT"
if [ $# -gt 0 ]; then
  if [ -f "$1" ]; then
    TARGET="$(cd "$(dirname "$1")" && pwd)/$(basename "$1")"
  elif [ -f "$REPO/$1" ]; then
    TARGET="$REPO/$1"
  else
    echo "ERROR: no such script: $1" >&2
    echo "       usage: ./run.sh [path/to/script.py] [args...]" >&2
    exit 1
  fi
  shift
fi

# --- run ---------------------------------------------------------------------
echo "==> $(basename "$TARGET")   (interpreter: $("$PYTHON_BIN" -c 'import sys; print(sys.executable)'))"
echo "    is the simulator running?  ros2 launch unitree_go2_sim unitree_go2_launch.py"
exec "$PYTHON_BIN" "$TARGET" "$@"
