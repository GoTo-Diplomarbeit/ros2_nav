"""
Tests for the mission state guard in NodeWrapper.

A test node publishes states on /mission_state exactly like the state node would,
and a small NodeWrapper inheritor records when _run() and the hooks are called.
"""
import time

import pytest
import rclpy
from std_msgs.msg import String

from goto.node_utils.node_wrapper import NodeWrapper
from goto.state.mission_state import MISSION_STATE_QOS, MISSION_STATE_TOPIC, State


class GuardedNode(NodeWrapper):
    """Minimal inheritor: only active while GUIDING, records every call."""

    active_states = frozenset({State.GUIDING})
    run_period_s = 0.05
    state_timeout_s = 0.5  # short, so the stale test does not take seconds

    def __init__(self):
        self.run_calls = 0
        self.entered = []
        self.exited = []
        super().__init__("guarded_test_node", self._setup_channels)

    def _setup_channels(self):
        pass

    def _run(self):
        self.run_calls += 1

    def on_state_enter(self, state):
        self.entered.append(state)

    def on_state_exit(self, state):
        self.exited.append(state)


# -- Setup code

@pytest.fixture
def rclpy_context():
    # Runs before test
    rclpy.init()
    yield
    # Runs after test
    rclpy.shutdown()


@pytest.fixture
def state_publisher_node(rclpy_context):
    # Plays the role of the state node
    test_node = rclpy.create_node("test_state_publisher")
    publisher = test_node.create_publisher(String, MISSION_STATE_TOPIC, MISSION_STATE_QOS)
    yield test_node, publisher
    test_node.destroy_node()


def spin_for(executor, seconds):
    """Lets ROS process messages and timers for the given time."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.01)


def make_executor(*wrappers_or_nodes):
    executor = rclpy.executors.SingleThreadedExecutor()
    for item in wrappers_or_nodes:
        executor.add_node(item.node if isinstance(item, NodeWrapper) else item)
    return executor


# -- Test code

def test_no_state_means_inactive(state_publisher_node):
    test_node, _ = state_publisher_node
    guarded = GuardedNode()
    executor = make_executor(guarded, test_node)

    spin_for(executor, 0.3)

    assert guarded.state is None
    assert not guarded.is_active()
    assert guarded.run_calls == 0
    guarded.destroy_node()


def test_runs_only_in_active_state(state_publisher_node):
    test_node, publisher = state_publisher_node
    guarded = GuardedNode()
    executor = make_executor(guarded, test_node)

    # IDLE is not in active_states -> _run() must not be called
    publisher.publish(String(data=State.IDLE.value))
    spin_for(executor, 0.3)
    assert guarded.state == State.IDLE
    assert guarded.run_calls == 0

    # GUIDING is -> _run() is called repeatedly
    publisher.publish(String(data=State.GUIDING.value))
    spin_for(executor, 0.3)
    assert guarded.is_active()
    assert guarded.run_calls > 0
    guarded.destroy_node()


def test_hooks_fire_once_per_change(state_publisher_node):
    test_node, publisher = state_publisher_node
    guarded = GuardedNode()
    executor = make_executor(guarded, test_node)

    publisher.publish(String(data=State.GUIDING.value))
    spin_for(executor, 0.2)
    # Same state again (like a heartbeat) -> no extra hook calls
    publisher.publish(String(data=State.GUIDING.value))
    spin_for(executor, 0.2)
    publisher.publish(String(data=State.IDLE.value))
    spin_for(executor, 0.2)

    assert guarded.entered == [State.GUIDING, State.IDLE]
    assert guarded.exited == [State.GUIDING]
    guarded.destroy_node()


def test_stale_state_becomes_unknown(state_publisher_node):
    test_node, publisher = state_publisher_node
    guarded = GuardedNode()
    executor = make_executor(guarded, test_node)

    publisher.publish(String(data=State.GUIDING.value))
    spin_for(executor, 0.2)
    assert guarded.is_active()

    # No heartbeat for longer than state_timeout_s -> state is forgotten, exit hook fires
    spin_for(executor, guarded.state_timeout_s + 0.3)
    assert guarded.state is None
    assert not guarded.is_active()
    assert guarded.exited == [State.GUIDING]

    # Stops calling _run() once stale
    calls_when_stale = guarded.run_calls
    spin_for(executor, 0.2)
    assert guarded.run_calls == calls_when_stale
    guarded.destroy_node()


def test_late_joiner_gets_current_state(state_publisher_node):
    test_node, publisher = state_publisher_node

    # State is published BEFORE the node exists - TRANSIENT_LOCAL must still deliver it
    publisher.publish(String(data=State.GUIDING.value))
    guarded = GuardedNode()
    executor = make_executor(guarded, test_node)
    spin_for(executor, 0.3)

    assert guarded.state == State.GUIDING
    guarded.destroy_node()


def test_unknown_state_is_ignored(state_publisher_node):
    test_node, publisher = state_publisher_node
    guarded = GuardedNode()
    executor = make_executor(guarded, test_node)

    publisher.publish(String(data=State.GUIDING.value))
    spin_for(executor, 0.2)
    publisher.publish(String(data="FLYING"))
    spin_for(executor, 0.2)

    assert guarded.state == State.GUIDING
    guarded.destroy_node()
