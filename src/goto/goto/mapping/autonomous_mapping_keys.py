"""
Keys for autonomous mapping. Runs in the terminal of scripts/build_run_sim_autonomous_mapping.sh and
replaces teleop_twist_keyboard there.

    p            pause / resume exploring
    i  j  l  ,   drive by hand: forward, turn left, turn right, backward
    k            stop moving
    x            finish: the script stops everything and saves the map (Ctrl+C does the same)

Driving by hand needs exploring to be paused, otherwise Nav2 and the keys fight over /cmd_vel. A drive key
pauses exploring first, and works once the explorer reports that the robot stands (press it again then).
A drive key moves the robot for HOLD_S, so keep it pressed to keep driving; the terminal repeats the key.
"""

import json
import os
import select
import signal
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from goto.mapping.autonomous_mapping_node import COMMAND_TOPIC, STATUS_QOS, STATUS_TOPIC
from goto.node_utils import node_runner
from goto.node_utils.node_wrapper import NodeWrapper

HELP = """Autonomous mapping - keys (this terminal):
  p        pause / resume exploring
  i j l ,  drive by hand: forward, left, right, back (pauses exploring first)
  k        stop moving
  x        finish: stop and save the map
  Ctrl+C   stop and save the map
"""

# key: (forward, turn left) as factors of the speeds
DRIVE_KEYS = {"i": (1, 0), ",": (-1, 0), "j": (0, 1), "l": (0, -1)}
HOLD_S = 0.6  # a drive key moves the robot this long; the terminal's key repeat keeps it going
TIMER_PERIOD_S = 0.05
EXPLORER_BUSY = {"exploring", "waiting", "pausing"}  # statuses in which Nav2 may still be driving

# What to tell the user when the status changes
MESSAGES = {
    "paused": "Paused. Drive with i j l , - press p to explore again, x to save the map.",
    "done": "No frontiers left. Press x to save the map, or drive to missed areas and press p to explore again.",
    "stuck": "Some frontiers could not be reached (red in rviz), or the robot is stuck. Free it with i j l , "
    "and press p to go on, or press x to save the map.",
    "time_limit": "Time limit reached. Press x to save the map, or p to explore for another round.",
    "pausing": "Stopping the robot ...",
}


class AutonomousMappingKeysNode(NodeWrapper):
    def __init__(self):
        super().__init__("autonomous_mapping_keys", self._setup_channels, follow_state=False)

    def _setup_channels(self):
        node = self.node
        self._speed = node.declare_parameter("speed", 0.3).value  # m/s
        self._turn = node.declare_parameter("turn", 0.5).value  # rad/s
        self._velocity_publisher = node.create_publisher(Twist, "/cmd_vel", 10)
        self._command_publisher = node.create_publisher(String, COMMAND_TOPIC, 10)
        node.create_subscription(String, STATUS_TOPIC, self._on_status, STATUS_QOS)

        self._status = ""  # "" until the explorer reported: keys then work like plain teleop
        self._drive = None  # the Twist being sent
        self._drive_until = 0.0
        self._was_driving = False

        # Key by key without Enter. Not for tests or CI, which have no terminal. cbreak (not raw) keeps
        # Ctrl+C working as a signal, so the run ends like with any other program.
        self._terminal_settings = None
        if sys.stdin.isatty():
            self._terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            print(HELP, flush=True)

        node.create_timer(TIMER_PERIOD_S, self._on_timer)

    def _on_timer(self) -> None:
        self._read_keys()
        self._send_velocity()

    def _read_keys(self) -> None:
        if self._terminal_settings is None:
            return
        file_descriptor = sys.stdin.fileno()
        try:
            # Non-blocking: this node has only one thread, and a blocked read would stop everything else
            while select.select([file_descriptor], [], [], 0)[0]:
                data = os.read(file_descriptor, 64)
                if not data:  # the terminal was closed
                    raise KeyboardInterrupt
                for key in data.decode(errors="ignore"):
                    self._on_key(key.lower())
        except OSError:  # a closed terminal can also show up as an I/O error
            raise KeyboardInterrupt from None

    def _on_key(self, key: str) -> None:
        if key == "p":
            self._command_publisher.publish(String(data="toggle"))
        elif key in DRIVE_KEYS:
            self._start_driving(*DRIVE_KEYS[key])
        elif key == "k":
            self._stop_driving()
        elif key == "x":
            print("Finishing - saving the map ...", flush=True)
            raise KeyboardInterrupt  # node_runner ends, then the script stops everything and saves the map

    def _start_driving(self, forward: float, left: float) -> None:
        if self._status in EXPLORER_BUSY:
            if self._status != "pausing":
                self._command_publisher.publish(String(data="pause"))
            print("Pausing exploration - press the key again once the robot stands.", flush=True)
            return
        self._drive = Twist()
        self._drive.linear.x = forward * self._speed
        self._drive.angular.z = left * self._turn
        self._drive_until = time.monotonic() + HOLD_S

    def _stop_driving(self) -> None:
        self._drive, self._drive_until = None, 0.0
        self._velocity_publisher.publish(Twist())
        self._was_driving = False

    def _send_velocity(self) -> None:
        if self._drive is not None and time.monotonic() < self._drive_until:
            self._velocity_publisher.publish(self._drive)
            self._was_driving = True
        elif self._was_driving:  # the key was released: one zero velocity, then silence
            self._velocity_publisher.publish(Twist())
            self._was_driving = False
            self._drive = None

    def _on_status(self, message: String) -> None:
        try:
            status = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if status["status"] == self._status:
            return
        self._status = status["status"]
        if self._status == "exploring":
            print(f">>> Exploring - {status['frontiers']} frontiers known", flush=True)
        elif self._status == "waiting":
            print(f">>> Waiting for {status['waiting_for'] or 'the explorer'} ...", flush=True)
        elif self._status in MESSAGES:
            print(f">>> {MESSAGES[self._status]}", flush=True)

    def destroy_node(self) -> None:
        if self._was_driving:
            self._velocity_publisher.publish(Twist())
        if self._terminal_settings is not None:  # give the terminal back as it was
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._terminal_settings)
            self._terminal_settings = None
        super().destroy_node()


def _stop_like_ctrl_c(_signal_number, _frame) -> None:
    raise KeyboardInterrupt


# Main function - Called when executing file
def main():
    # A closed terminal (SIGHUP) or a plain kill (SIGTERM) end the node like Ctrl+C, so the terminal is restored
    signal.signal(signal.SIGTERM, _stop_like_ctrl_c)
    signal.signal(signal.SIGHUP, _stop_like_ctrl_c)
    node_runner.run_node(lambda: AutonomousMappingKeysNode())


if __name__ == "__main__":
    main()
