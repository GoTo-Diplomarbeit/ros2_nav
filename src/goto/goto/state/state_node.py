"""
State Node - the single source of truth for the mission state.

It is the ONLY node that changes the state. Other nodes never set it directly: they
send a request (later: events like "destination selected" or "person lost") and this
node decides. Every other node reads the state through NodeWrapper.

For now any valid request is accepted, so the guard can be tried out from a terminal:
    ros2 topic pub --once /mission_state_request std_msgs/String "data: GUIDING"
    ros2 topic echo /mission_state
The real transition rules (which state may follow which) come with the state machine.
"""

from std_msgs.msg import String

from goto.node_utils import node_runner
from goto.node_utils.node_wrapper import NodeWrapper
from goto.state.mission_state import (HEARTBEAT_PERIOD_S, MISSION_STATE_QOS, MISSION_STATE_REQUEST_TOPIC,
                                         MISSION_STATE_TOPIC, State, parse_state)


class StateNode(NodeWrapper):
    """
    Holds the mission state and publishes it on every change and as a heartbeat.
    """

    def __init__(self, initial_state: State = State.STARTUP) -> None:
        """
        Args:
            initial_state: The state published right after startup.
        """
        # follow_state=False: this node owns the state, it must not follow its own topic
        super().__init__("mission_state", lambda: self._setup_channels(initial_state), follow_state=False)

    def set_state(self, new_state: State) -> None:
        """
        Changes the mission state and publishes it immediately.
        Args:
            new_state: The new state.
        """
        if new_state == self.state:
            return

        self.node.get_logger().info(f"Mission state: {self.state.value} -> {new_state.value}")
        self._change_state(new_state)
        self._publish_state()

    # --
    # -- Private class functions
    # --

    def _setup_channels(self, initial_state: State) -> None:
        """
        Sets up the state publisher, the request subscription and the heartbeat.
        Returns: None

        """
        self._state_publisher = self.node.create_publisher(String, MISSION_STATE_TOPIC, MISSION_STATE_QOS)

        self._request_subscription = self.node.create_subscription(
            String, MISSION_STATE_REQUEST_TOPIC, self._on_request, 10)

        # Re-publish regularly, so the other nodes notice if this node dies
        self._heartbeat_timer = self.node.create_timer(HEARTBEAT_PERIOD_S, self._publish_state)

        # Publish the initial state right away - TRANSIENT_LOCAL keeps it for late joiners
        self._change_state(initial_state)
        self._publish_state()

    def _on_request(self, message: String) -> None:
        """
        Called for every state change request.
        """
        requested_state = parse_state(message.data)
        if requested_state is None:
            known = ", ".join(state.value for state in State)
            self.node.get_logger().warning(f"Ignoring unknown state request '{message.data}'. Known: {known}")
            return

        self.set_state(requested_state)

    def _publish_state(self) -> None:
        self._state_publisher.publish(String(data=self.state.value))


# Main function - Called when executing file
def main():
    # Run the state node
    node_runner.run_node(lambda: StateNode())


if __name__ == '__main__':
    main()
