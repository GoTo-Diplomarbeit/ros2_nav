import time

import pytest
import rclpy
from std_msgs.msg import String

from goto.state.mission_state import (MISSION_STATE_QOS, MISSION_STATE_REQUEST_TOPIC, MISSION_STATE_TOPIC,
                                         State)
from goto.state.state_node import StateNode


# -- Setup code

@pytest.fixture
def rclpy_context():
    # Runs before test
    rclpy.init()
    yield
    # Runs after test
    rclpy.shutdown()


@pytest.fixture
def state_setup(rclpy_context):
    state_node = StateNode()

    # Plays the role of the other nodes: sends requests, records the published states
    test_node = rclpy.create_node("test_helper")
    request_publisher = test_node.create_publisher(String, MISSION_STATE_REQUEST_TOPIC, 10)
    received_states = []
    test_node.create_subscription(String, MISSION_STATE_TOPIC,
                                  lambda message: received_states.append(message.data), MISSION_STATE_QOS)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(state_node.node)
    executor.add_node(test_node)

    yield state_node, request_publisher, received_states, executor

    state_node.destroy_node()
    test_node.destroy_node()


def spin_for(executor, seconds):
    """Lets ROS process messages and timers for the given time."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)


# -- Test code

def test_publishes_initial_state(state_setup):
    state_node, _, received_states, executor = state_setup

    spin_for(executor, 0.3)

    assert state_node.state == State.STARTUP
    assert received_states[-1] == State.STARTUP.value


def test_valid_request_changes_state(state_setup):
    state_node, request_publisher, received_states, executor = state_setup
    spin_for(executor, 0.2)

    request_publisher.publish(String(data=State.GUIDING.value))
    spin_for(executor, 0.3)

    assert state_node.state == State.GUIDING
    assert received_states[-1] == State.GUIDING.value


def test_unknown_request_is_ignored(state_setup):
    state_node, request_publisher, _, executor = state_setup
    spin_for(executor, 0.2)

    request_publisher.publish(String(data="FLYING"))
    spin_for(executor, 0.3)

    assert state_node.state == State.STARTUP


def test_heartbeat_repeats_state(state_setup):
    _, _, received_states, executor = state_setup

    # Initial publish + at least one heartbeat (HEARTBEAT_PERIOD_S = 1 s)
    spin_for(executor, 1.5)

    assert received_states.count(State.STARTUP.value) >= 2


def test_state_node_does_not_follow_itself(state_setup):
    state_node, _, _, _ = state_setup

    # follow_state=False -> the only subscription is the request topic
    subscribed_topics = [topic for topic, _ in state_node.node.get_subscriber_names_and_types_by_node(
        "mission_state", "/")]
    assert MISSION_STATE_TOPIC not in subscribed_topics
