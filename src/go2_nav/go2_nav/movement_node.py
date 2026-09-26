import rclpy
from geometry_msgs.msg import Twist


class MovementNode:
    """
    Bridge between planner output and the actual movement target.
    Subscribes to /planner_cmd_vel; forwards to /cmd_vel for simulator use or translates into Sport Mode SDK calls for the real robot.
    """

    # -- Class functions

    def __init__(self, use_sim: bool = True, timer_period_sec: float | int = 0.02) -> None:
        """
        Initialize the movement node and starts the movement timer.
        Args:
            use_sim (bool, optional): Whether to use simulator or not.
            timer_period_sec (float | int): How often the timer publishes the velocity command in seconds.
        """
        # Sets the flag if using sim or not
        self._use_sim = use_sim
        # Sets the timer period in seconds
        self._timer_period_sec = timer_period_sec

        # Movement Node for the whole Movement class
        self.movement_node = rclpy.create_node("movement")

        # Setup all channels
        self._setup_channels()

    def _setup_channels(self):
        self._planner_subscription = self.movement_node.create_subscription(Twist,
                                                                            "/planner_cmd_vel",
                                                                            self._publish_movement_command, 10)

        if self._use_sim:
            # Movement command publisher - Used for publishing movement Twist commands
            self._movement_command_publisher = self.movement_node.create_publisher(Twist, "/cmd_vel", 10)

        else:
            raise NotImplementedError("Functions for translating to real robot not yet implemented!")

    def destroy(self) -> None:
        # Destroy the movement node
        self.movement_node.destroy_node()

    # -- Movement commands

    def _publish_movement_command(self, movement_command: Twist) -> None:
        """
        Publishes the movement command to the movement node - Only used inside timer.
        Returns: None

        """
        # If not using the simulator -> Throw
        if not self._use_sim:
            # TODO: Implement translation layer
            raise ValueError("You can only use this function when using the simulator!")

        self._movement_command_publisher.publish(movement_command)


# Main function - Called when executing file
if __name__ == '__main__':
    # Initializes rclpy
    rclpy.init()
    # Create a movement node
    movement_node = MovementNode()

    try:
        # Spin up the movement node indefinitely
        rclpy.spin(movement_node.movement_node)
    finally:
        # Destroy the movement node when finished
        movement_node.destroy()
        # Shutdown rclpy
        rclpy.shutdown()
