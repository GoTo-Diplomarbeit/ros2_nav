#!/usr/bin/env python3
import os
import signal
import subprocess
import sys
import time

SETUP = (
    "source /opt/ros/jazzy/setup.bash && "
    "source /unitree_go2_sim/install/setup.bash && "
    "source /ws/install/setup.bash"
)

procs = []


def start(cmd):
    """Start a command in the background in its own process group."""
    p = subprocess.Popen(["bash", "-c", f"{SETUP} && exec {cmd}"],
                         start_new_session=True)
    procs.append(p)
    return p


def stop_all():
    for p in reversed(procs):
        try:
            os.killpg(p.pid, signal.SIGINT)  # like Ctrl+C
        except ProcessLookupError:
            pass
    deadline = time.time() + 15
    for p in procs:
        try:
            p.wait(timeout=max(0, deadline - time.time()))
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGKILL)


try:
    start("ros2 launch unitree_go2_sim unitree_go2_launch.py "
          "static_map_to_odom:=false rviz:=false")
    time.sleep(30)

    start("ros2 launch goto navigation_rtabmap.launch.py")
    time.sleep(10)

    # Teleop runs in the foreground so it receives your keys
    subprocess.run(["bash", "-c",
                    f"{SETUP} && ros2 run teleop_twist_keyboard teleop_twist_keyboard"])
except KeyboardInterrupt:
    pass
finally:
    stop_all()

sys.exit(0)
