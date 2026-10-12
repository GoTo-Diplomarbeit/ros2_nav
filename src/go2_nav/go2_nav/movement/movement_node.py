from typing import Optional

from geometry_msgs.msg import Twist

from go2_nav.movement.movement_backend import MovementBackendType, GazeboBackend, Go2Backend, NetworkInterface
from go2_nav.node_utils import node_runner
from go2_nav.node_utils.node_wrapper import NodeWrapper


class MovementNode(NodeWrapper):
    """
    Bridge between planner output and the actual movement target.
    Subscribes to /planner_cmd_vel; forwards to /cmd_vel for simulator use or translates into Sport Mode SDK calls for the real robot.
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

    # --
    # -- Private class functions
    # --

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
        self._planner_subscription = self.node.create_subscription(Twist,
                                                                            "/planner_cmd_vel",
                                                                            self._publish_movement_command, 10)

        # User chose simulator -> Build simulator Node
        if movement_backend_type == MovementBackendType.GAZEBO:
            # Movement command publisher - Used for publishing movement Twist commands
            _movement_command_publisher = self.node.create_publisher(Twist, "/cmd_vel", 10)

            # Create the required backend
            self.movement_backend = GazeboBackend(_movement_command_publisher)

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
        # Send the movement command to the backend
        self.movement_backend.send(movement_command)


# Main function - Called when executing file
def main():
    # Run the movement node - the backend is chosen by the "backend" ROS parameter
    node_runner.run_node(lambda: MovementNode())


if __name__ == '__main__':
    main()
