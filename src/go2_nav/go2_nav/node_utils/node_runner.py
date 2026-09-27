from typing import Callable

import rclpy

from go2_nav.node_utils.node_wrapper import NodeWrapper


def run_node(node_wrapper_factory: Callable[[], NodeWrapper]) -> None:
    """
    Runs a node using rclpy.
    Args:
        node_wrapper_factory (Callable[[], NodeWrapper]): A factory function to create the node wrapper.

    Returns: None

    """
    # Initializes rclpy
    rclpy.init()

    node_wrapper = node_wrapper_factory()

    try:
        # Spin up the movement node indefinitely
        rclpy.spin(node_wrapper.node)
    finally:
        # Destroy the movement node when finished
        node_wrapper.destroy_node()
        # Shutdown rclpy
        rclpy.shutdown()
