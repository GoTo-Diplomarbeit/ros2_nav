"""
Movement Node
"""

import copy
import math
from typing import Optional

from geometry_msgs.msg import Pose, Twist

from goto.movement.movement_backend import (
    GazeboBackend,
    Go2Backend,
    MovementBackendType,
    NetworkInterface,
)
from goto.node_utils import node_runner
from goto.node_utils.node_wrapper import NodeWrapper
from goto.state.mission_state import State


class MovementNode(NodeWrapper):
    """
    Bridge between planner output and the actual movement target.
    Subscribes to /planner_cmd_vel; forwards to /cmd_vel for simulator use or
    translates into Sport Mode SDK calls for the real robot.
    """

    def __init__(self, movement_backend_type: Optional[MovementBackendType] = None) -> None:
        """
        Initialize the movement node and starts the movement timer.
        Args:
            movement_backend_type (Optional[MovementBackendType]): Whether to use simulator or real robot.
                If None, the "backend" ROS parameter ("gazebo" or "go2") decides.
        """

        # Movement Node for the whole Movement class
        super().__init__("movement", lambda: self._setup_channels(movement_backend_type))

        # Variable for saving newest incoming Twist command
        self._last_command: Optional[Twist] = None

        self._target_yaw: Optional[float] = None

        self._kp: float = 2.0

        self._pitch_target: float = -math.radians(5)

        self._current_pitch: float = 0.0

        self._photo_activ: bool = False

        self._ramp_start = self.node.get_clock().now()

        # Creates a Timer which is called every 0.01s
        self._timer = self.node.create_timer(0.01, self._on_timer)

        self.active_states = frozenset({State.PERSON_LOST})

    def _run(self) -> None:
        """
        Lets you choose which methode should be called at which STATE
        """
        match self.state:
            case State.PERSON_LOST:
                self._photo_pose()

    # --
    # -- Private class functions
    # --

    def _start_ramp(self) -> None:
        """
        Calculates the perfect s shaped acceleration for the tilt
        """
        if not self._photo_activ:
            return
        duration = 2.0

        t = (self.node.get_clock().now() - self._ramp_start).nanoseconds / 1e9
        s = min(max(t / duration, 0.0), 1.0)
        s = s * s * (3.0 - 2.0 * s)  # smoothstep (S-shaped)
        self.movement_backend.send_tilt_command(self._pitch_target * s)

    def _photo_pose(self) -> None:
        """
        Methode which sends the commands for the position of the robot to make a proper photo of a person
        """
        if self._photo_activ:
            return

        self._photo_activ = True
        self._ramp_start = self.node.get_clock().now()

        last = self._last_command if self._last_command is not None else Twist()
        command_to_send = copy.deepcopy(last)
        command_to_send.linear.x = 0.0
        command_to_send.linear.y = 0.0

        self._last_command = command_to_send
        self.movement_backend.send_movement_command(command_to_send)

        self._start_ramp()

    def _on_timer(self) -> None:
        """
        Method which is called every 0.05s, corrects the angular values of the robot
        """
        self._start_ramp()

        if self._last_command is None:
            return

        # Check if last command is Twist with 0 0 0, 0 0 0
        if self._last_command == Twist():
            command_to_send = Twist()  # Send one 0 command to stop
        else:
            last = self._last_command
            if last is None:
                return

            command_to_send = copy.deepcopy(last)
            yaw = self.movement_backend.get_yaw()
            target_yaw = self._target_yaw

            # for returning the angular values in the log
            if yaw is not None:
                self.node.get_logger().info(f"Korrektur aktiv -> YAW: {yaw:.3f}")

            if abs(last.angular.z) < 0.01 and target_yaw is not None and yaw is not None:
                error_yaw = target_yaw - yaw
                true_error = math.atan2(math.sin(error_yaw), math.cos(error_yaw))
                command_to_send.angular.z += true_error * self._kp

            # needed to read the target and the error
            # self.node.get_logger().info(
            # f"Korrektur aktiv -> TARGET: {self._target_yaw:.3f} | YAW: {yaw:.3f} | TRUE_ERROR:
            # {true_error:.3f}")

        self.movement_backend.send_movement_command(command_to_send)

    def _setup_channels(self, movement_backend_type: Optional[MovementBackendType]) -> None:
        """
        Sets up the subscription and publisher channels for the movement node.
        Returns: None

        """
        # Parameters - can be set from the launch file or with --ros-args -p
        self.node.declare_parameter("backend", MovementBackendType.GAZEBO.name.lower())

        self.node.declare_parameter("network_interface", "eth0")

        # Explicit argument wins, otherwise use the "backend" parameter
        if movement_backend_type is None:
            movement_backend_type = MovementBackendType[self.node.get_parameter("backend").value.upper()]

        # The selected movement backend type
        self._planner_subscription = self.node.create_subscription(
            Twist, "/planner_cmd_vel", self._publish_movement_command, 10
        )

        # User chose simulator -> Build simulator Node
        if movement_backend_type == MovementBackendType.GAZEBO:
            # Movement command publisher - Used for publishing movement Twist commands

            _movement_command_publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)
            _pose_command_publisher = self.node.create_publisher(Pose, "/body_pose", 10)
            # Create the required backend
            self.movement_backend = GazeboBackend(_movement_command_publisher, _pose_command_publisher, self.node)

        else:
            # The required network interface
            _network_interface = NetworkInterface(self.node.get_parameter("network_interface").value)

            # Create the required backend
            self.movement_backend = Go2Backend(_network_interface)

    def _publish_movement_command(self, movement_command: Twist) -> None:
        """
        Publishes the movement command to the movement node - Only used inside timer.
        Returns: None

        """
        # Compares the new Twist command to the old one
        if self._last_command != movement_command:
            # Calls the current yaw values and saves them
            yaw = self.movement_backend.get_yaw()
            self._target_yaw = yaw

        # Send the movement command to the backend
        self._last_command = movement_command


# Main function - Called when executing file
def main():
    # Run the movement node - the backend is chosen by the "backend" ROS parameter
    node_runner.run_node(lambda: MovementNode())


if __name__ == "__main__":
    main()
