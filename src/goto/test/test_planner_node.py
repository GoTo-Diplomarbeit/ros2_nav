import json
import math

import pytest
import rclpy

from goto.planner.planner_node import PlannerNode, quaternion_from_yaw, status_message

# -- Setup code


# Reusable code that can be used in multiple tests
@pytest.fixture
def rclpy_context():
    # Runs before test
    rclpy.init()
    yield
    # Runs after test
    rclpy.shutdown()


# -- Test code


def test_planner_node_starts(rclpy_context):
    # The planner is still empty - this only guards against it crashing on startup
    # (it used to, because it did not pass _setup_channels to NodeWrapper)
    planner_node = PlannerNode()

    assert planner_node.node.get_name() == "planner"

    planner_node.destroy_node()


@pytest.mark.parametrize(
    "yaw_degrees, expected_z, expected_w",
    [(0, 0.0, 1.0), (90, 0.7071, 0.7071), (180, 1.0, 0.0)],
)
def test_quaternion_turns_the_robot_by_the_room_heading(yaw_degrees, expected_z, expected_w):
    z, w = quaternion_from_yaw(math.radians(yaw_degrees))

    assert (z, w) == pytest.approx((expected_z, expected_w), abs=1e-4)


def test_status_message_is_json_for_the_app():
    assert json.loads(status_message("1.12", "driving", 3.14159)) == {
        "room": "1.12",
        "status": "driving",
        "distance_left": 3.1,
    }
    assert json.loads(status_message("1.12", "arrived")) == {"room": "1.12", "status": "arrived"}
