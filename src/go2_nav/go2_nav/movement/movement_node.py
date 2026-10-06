from geometry_msgs.msg import Twist

from go2_nav.movement.movement_backend import MovementBackendType, GazeboBackend, Go2Backend, NetworkInterface
from go2_nav.node_utils import node_runner
from go2_nav.node_utils.node_wrapper import NodeWrapper

from typing import Optional
import copy
import math

class MovementNode(NodeWrapper):
    """
    Bridge between planner output and the actual movement target.
    Subscribes to /planner_cmd_vel; forwards to /cmd_vel for simulator use or translates into Sport Mode SDK calls for the real robot.
    """

    def __init__(self, movement_backend_type: MovementBackendType) -> None:
        """
        Initialize the movement node and starts the movement timer.
        Args:
            movement_backend_type (MovementBackendType): Whether to use simulator or real robot.
        """

        # Movement Node for the whole Movement class
        super().__init__("movement", lambda: self._setup_channels(movement_backend_type))

        # Variable for saving newest incoming Twist command
        self._last_command: Optional[Twist] = None

        self._target_yaw: Optional[float] = None

        self._kp: float = 0.5

        # Creates a Timer which is called every 0.05s
        self._timer = self.node.create_timer(0.05, self._on_timer)


    # --
    # -- Private class functions
    # --

    def _on_timer(self) -> None:

        """
        Method which is called every 0.05s
        """

        if self._last_command is None:
            return

        command_to_send = copy.deepcopy(self._last_command)

        yaw = self.movement_backend.get_yaw()

        if yaw is not None:
            self.node.get_logger().info(f"YAW: {yaw:.3f}")

        if self._target_yaw is not None:
            if yaw is not None:
                error_yaw = self._target_yaw - yaw
                true_error = math.atan2(math.sin(error_yaw), math.cos(error_yaw))
                command_to_send.angular.z += true_error * self._kp


        self.movement_backend.send(command_to_send)


    def _setup_channels(self, movement_backend_type: MovementBackendType) -> None:
        """
        Sets up the subscription and publisher channels for the movement node.
        Returns: None

        """
        # The selected movement backend type
        self._planner_subscription = self.node.create_subscription(Twist,
                                                                            "/planner_cmd_vel",
                                                                            self._publish_movement_command, 10)

        # User chose simulator -> Build simulator Node
        if movement_backend_type == MovementBackendType.GAZEBO:
            # Movement command publisher - Used for publishing movement Twist commands
            _movement_command_publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)

            # Create the required backend
            self.movement_backend = GazeboBackend(_movement_command_publisher, self.node)

        else:
            # The required network interface
            _network_interface = NetworkInterface("eht0")

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
    _required_backend = MovementBackendType.GAZEBO

    # Run the movement node
    node_runner.run_node(lambda: MovementNode(_required_backend))


if __name__ == '__main__':
    main()
