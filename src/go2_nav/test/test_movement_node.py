import pytest
import rclpy
from geometry_msgs.msg import Twist

from go2_nav.movement_node import MovementNode


# -- Setup code

# Reusable code that can be used in multiple tests
@pytest.fixture
def rclpy_context():
    # Runs before test
    rclpy.init()
    yield
    # Runs after test
    rclpy.shutdown()


@pytest.fixture
def movement_setup(rclpy_context):
    # Create the movementNode to test
    movement_node = MovementNode(use_sim=True)
    # Create a testNode to simulate the planner and sim/robot
    test_node = rclpy.create_node("test_helper")

    # Create a planner publisher
    planner_publisher = test_node.create_publisher(Twist, "/planner_cmd_vel", 10)

    received_movement_commands = []
    test_node.create_subscription(Twist, "/cmd_vel", received_movement_commands.append, 10)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(movement_node.movement_node)
    executor.add_node(test_node)

    # Pass the values to the tests
    yield movement_node, test_node, planner_publisher, received_movement_commands, executor

    movement_node.destroy_node()
    test_node.destroy_node()


# -- Test code

def test_sim_forwarding(movement_setup):
    movement_node, test_node, planner_publisher, received_movement_commands, executor = movement_setup

    movement_command = Twist()
    movement_command.linear.x = 0.5
    movement_command.angular.z = 0.1

    planner_publisher.publish(movement_command)

    for _ in range(10):
        executor.spin_once(timeout_sec=0.1)
        if received_movement_commands:
            break

    assert len(received_movement_commands) == 1
    assert received_movement_commands[0].linear.x == 0.5
    assert received_movement_commands[0].angular.z == 0.1
