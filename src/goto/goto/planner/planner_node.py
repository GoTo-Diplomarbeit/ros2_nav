"""
Planner Node - drives the robot to the room the visitor selected in the app.

    /goto_room          (std_msgs/String) room name as in maps/<world>_rooms.yaml, e.g. "1.12".
                        An empty string stops the current trip.
    /navigation_status  (std_msgs/String, JSON; the last one is kept for late subscribers)
                        {"room": "1.12", "status": "driving", "distance_left": 3.2}
                        status: driving, arrived, failed, canceled, unknown_room, nav2_not_ready

The room is looked up in the room file (made with scripts/build_run_define_rooms.sh) and sent to Nav2
as a NavigateToPose action. Unlike the /goal_pose topic, the action reports the distance left and
whether the goal was reached. For the state machine it requests GUIDING when it starts driving and
ARRIVED or BLOCKED at the end. It doesn't check the state yet: a room is accepted in any state.

Try it while the navigation script runs:
    ros2 topic pub --once -w 1 /goto_room std_msgs/String "data: '1.12'"   (-w 1: wait for this node first)
    ros2 topic echo /navigation_status
"""

import json
import math
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from goto.node_utils import node_runner
from goto.node_utils.node_wrapper import NodeWrapper
from goto.rooms.room_file import load_rooms
from goto.state.mission_state import MISSION_STATE_REQUEST_TOPIC, State

GOTO_ROOM_TOPIC = "/goto_room"
NAVIGATION_STATUS_TOPIC = "/navigation_status"
DEFAULT_ROOMS_FILE = "/ws/maps/default_rooms.yaml"  # the "default" world in the Docker container
FEEDBACK_PERIOD_S = 1.0  # how often "distance_left" is reported while driving

# Keep the last status for late subscribers (an app that connects later still sees it)
STATUS_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    """The (z, w) part of the quaternion for a heading on the floor (x and y stay 0)."""
    return math.sin(yaw / 2), math.cos(yaw / 2)


def status_message(room: str, status: str, distance_left: float | None = None) -> str:
    """The JSON text sent on /navigation_status."""
    message = {"room": room, "status": status}
    if distance_left is not None:
        message["distance_left"] = round(distance_left, 1)
    return json.dumps(message)


class PlannerNode(NodeWrapper):
    def __init__(self):
        super().__init__("planner", self._setup_channels)

    def _setup_channels(self):
        self._rooms_file = self.node.declare_parameter("rooms_file", DEFAULT_ROOMS_FILE).value
        self._navigator = ActionClient(self.node, NavigateToPose, "navigate_to_pose")
        self._status_publisher = self.node.create_publisher(String, NAVIGATION_STATUS_TOPIC, STATUS_QOS)
        self._state_request_publisher = self.node.create_publisher(String, MISSION_STATE_REQUEST_TOPIC, 10)
        self.node.create_subscription(String, GOTO_ROOM_TOPIC, self._on_goto_room, 10)

        self._room = ""  # the room of the current trip, "" = no trip
        self._goal_handle = None  # Nav2's handle of the current trip, None until Nav2 accepted it
        self._last_feedback_time = 0.0

    def _on_goto_room(self, message: String) -> None:
        room_name = message.data.strip()
        if not room_name:
            self._cancel_trip()
            return

        try:
            rooms = load_rooms(self._rooms_file)  # read every time, so newly defined rooms work at once
        except (OSError, ValueError) as error:
            self.node.get_logger().error(f"Can't read the room file {self._rooms_file}: {error}")
            self._publish_status(room_name, "failed")
            return

        room = rooms.get(room_name)
        if room is None:
            self.node.get_logger().warning(f"Unknown room '{room_name}'. Known: {', '.join(rooms) or 'none'}")
            self._publish_status(room_name, "unknown_room")
            return
        if not self._navigator.server_is_ready():
            self.node.get_logger().warning("Nav2 isn't running - can't drive anywhere")
            self._publish_status(room_name, "nav2_not_ready")
            return

        # A new room replaces the current trip. No cancel: Nav2 replaces a running goal by itself, and
        # a cancel arriving together with the new goal could stop that one too. Forgetting the old
        # handle makes _on_result ignore the old trip's result.
        self._goal_handle = None

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"  # no time stamp: Nav2 then uses the newest data
        goal.pose.pose.position.x = room.goal_x
        goal.pose.pose.position.y = room.goal_y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = quaternion_from_yaw(room.goal_yaw)

        self._room = room.name
        self.node.get_logger().info(f"Driving to '{room.name}' ({room.goal_x:.2f}, {room.goal_y:.2f})")
        sent = self._navigator.send_goal_async(goal, feedback_callback=self._on_feedback)
        sent.add_done_callback(lambda future, name=room.name: self._on_goal_response(future, name))

    def _on_goal_response(self, future, room_name: str) -> None:
        goal_handle = future.result()
        if room_name != self._room:  # replaced or stopped while Nav2 was answering
            if goal_handle.accepted and not self._room:  # stopped: don't let it drive
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self._room = ""
            self._publish_status(room_name, "failed")
            return

        self._goal_handle = goal_handle
        self._publish_status(room_name, "driving")
        self._request_state(State.GUIDING)
        goal_handle.get_result_async().add_done_callback(
            lambda future, handle=goal_handle: self._on_result(future, handle)
        )

    def _on_feedback(self, feedback_message) -> None:
        # Only for the current trip, and at most once per FEEDBACK_PERIOD_S
        if self._goal_handle is None or feedback_message.goal_id != self._goal_handle.goal_id:
            return
        if time.monotonic() - self._last_feedback_time < FEEDBACK_PERIOD_S:
            return
        self._last_feedback_time = time.monotonic()
        self._publish_status(self._room, "driving", feedback_message.feedback.distance_remaining)

    def _on_result(self, future, goal_handle) -> None:
        if goal_handle is not self._goal_handle:  # an old trip that was replaced or stopped
            return
        room, self._room, self._goal_handle = self._room, "", None

        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._publish_status(room, "arrived")
            self._request_state(State.ARRIVED)
        elif status == GoalStatus.STATUS_CANCELED:  # canceled by another program
            self._publish_status(room, "canceled")
            self._request_state(State.IDLE)
        else:
            # ABORTED: Nav2 couldn't reach it - or another goal replaced it, e.g. "2D Goal Pose" in
            # rviz; Nav2 doesn't tell the two apart, so that also counts as failed here
            self.node.get_logger().warning(f"Nav2 couldn't reach '{room}'")
            self._publish_status(room, "failed")
            self._request_state(State.BLOCKED)

    def _cancel_trip(self) -> None:
        """Stops the current trip (an empty /goto_room message), also one Nav2 hasn't accepted yet."""
        if not self._room:
            return
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        # Not accepted yet: _on_goal_response sees the empty _room and cancels it then
        self.node.get_logger().info(f"Trip to '{self._room}' canceled")
        self._publish_status(self._room, "canceled")
        self._request_state(State.IDLE)
        self._room, self._goal_handle = "", None

    def _publish_status(self, room: str, status: str, distance_left: float | None = None) -> None:
        self._status_publisher.publish(String(data=status_message(room, status, distance_left)))

    def _request_state(self, state: State) -> None:
        # Only a request: the state node decides (see goto/state/README.md)
        self._state_request_publisher.publish(String(data=state.value))


# Main function - Called when executing file
def main():
    # Run the planner node
    node_runner.run_node(lambda: PlannerNode())


if __name__ == "__main__":
    main()
