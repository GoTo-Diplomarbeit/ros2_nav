import pytest
import rclpy

from goto.planner.planner_node import PlannerNode

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
