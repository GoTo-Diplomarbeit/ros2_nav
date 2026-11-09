"""
Mission state shared by all nodes.

The state node (state_node.py) is the single source of truth: it is the ONLY node
that changes the state. It publishes it on MISSION_STATE_TOPIC whenever it changes,
plus once per HEARTBEAT_PERIOD_S so the other nodes can tell it is still alive.
Every other node only reads it (NodeWrapper does that for them).

Everything about the state lives in this one file, so no node can misspell a state
name or use different QoS settings.
"""

from enum import Enum
from typing import Optional

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class State(Enum):
    """
    All states of the guide-dog mission. The value is the text sent over ROS
    (std_msgs/String), so it can be read with "ros2 topic echo /mission_state".
    """

    STARTUP = "STARTUP"  # waiting for localization and Nav2
    IDLE = "IDLE"  # standing, waiting for a destination
    CHECK_REQUEST = "CHECK_REQUEST"  # is the destination known and reachable?
    REGISTER_PERSON = "REGISTER_PERSON"  # taking a photo of the person to guide
    GUIDING = "GUIDING"  # walking to the destination with the person
    PERSON_FELL_BEHIND = "PERSON_FELL_BEHIND"  # paused, waiting for the person to catch up
    PERSON_LOST = "PERSON_LOST"  # person gone for too long
    BLOCKED = "BLOCKED"  # Nav2 could not plan or reach the goal
    ARRIVED = "ARRIVED"  # destination reached
    RETURNING_HOME = "RETURNING_HOME"  # walking back to the start point
    EMERGENCY_STOP = "EMERGENCY_STOP"  # everything stopped, manual reset only
    ERROR = "ERROR"  # startup failed, manual restart only


def parse_state(text: str) -> Optional[State]:
    """
    Turns the text of a state message back into a State.
    Args:
        text: The state name, e.g. "GUIDING".
    Returns: The matching State, or None if the text is not a known state.
    """
    try:
        return State(text)
    except ValueError:
        return None


# The current state, published by the state node only
MISSION_STATE_TOPIC = "/mission_state"

# Requests to change the state (for testing from a terminal until the state machine
# exists), e.g.:  ros2 topic pub --once /mission_state_request std_msgs/String "data: GUIDING"
MISSION_STATE_REQUEST_TOPIC = "/mission_state_request"

# The state node re-publishes the current state this often, even if nothing changed
HEARTBEAT_PERIOD_S = 1.0

# A node treats the state as unknown after this long without a message - e.g. when
# the state node crashed. Two missed heartbeats, so one late message is tolerated.
STATE_TIMEOUT_S = 2.0 * HEARTBEAT_PERIOD_S

# TRANSIENT_LOCAL = the last message is kept for late joiners: a node that starts
# (or restarts) after the state was published still gets the current state at once,
# instead of waiting for the next heartbeat. Both sides must use this profile.
MISSION_STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
