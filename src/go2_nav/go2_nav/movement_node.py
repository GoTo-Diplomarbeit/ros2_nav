import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from go2_nav.node_utils import node_runner
from go2_nav.node_utils.node_wrapper import NodeWrapper


class MovementNode(NodeWrapper):
    """
    Bridge between planner output and the actual movement target.
    Subscribes to /planner_cmd_vel; forwards to /cmd_vel for simulator use or translates into Sport Mode SDK calls for the real robot.
    """

    def __init__(self, use_sim: bool = True) -> None:
        """
        Initialize the movement node and starts the movement timer.
        Args:
            use_sim (bool, optional): Whether to use simulator or not.
        """
        # Sets the flag if using sim or not
        self._use_sim = use_sim

        # Movement Node for the whole Movement class
        self.movement_node = rclpy.create_node("movement")

        # Setup all channels
        self._setup_channels()

    # --
    # -- Private class functions
    # --

    def _setup_channels(self) -> None:
        """
        Sets up the subscription and publisher channels for the movement node.
        Returns: None

        """
        self._planner_subscription = self.movement_node.create_subscription(Twist,
                                                                            "/planner_cmd_vel",
                                                                            self._publish_movement_command, 10)

        if self._use_sim:
            # Movement command publisher - Used for publishing movement Twist commands
            self._movement_command_publisher = self.movement_node.create_publisher(Twist, "/cmd_vel", 10)

        else:
            # TODO: Implement translation layer
            raise NotImplementedError("Functions for translating to real robot not yet implemented!")

    def _publish_movement_command(self, movement_command: Twist) -> None:
        """
        Publishes the movement command to the movement node - Only used inside timer.
        Returns: None

        """
        if self._use_sim:
            self._movement_command_publisher.publish(movement_command)

        else:
            # TODO: Implement translation layer
            raise NotImplementedError("Functions for translating to real robot not yet implemented!")

    # --
    # -- Public class functions
    # --

    @property
    def node(self) -> Node:
        """
        Function to return the actual movement node.
        Returns: The movement node.

        """
        return self.movement_node

    def destroy_node(self) -> None:
        """
        Destroys the movement node.
        Returns: None

        """
        self.movement_node.destroy_node()


# Main function - Called when executing file
def main():
    # Run the movement node
    node_runner.run_node(lambda: MovementNode(use_sim=True))


if __name__ == '__main__':
    main()
