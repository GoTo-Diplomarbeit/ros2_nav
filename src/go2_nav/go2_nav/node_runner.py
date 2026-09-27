import rclpy

from go2_nav.node_wrapper import NodeWrapper


def run_node(node_wrapper: NodeWrapper) -> None:
    """
    Runs a node using rclpy.
    Args:
        node_wrapper (NodeWrapper): The node wrapper from which node should be run.

    Returns: None

    """
    # Initializes rclpy
    rclpy.init()

    try:
        # Spin up the movement node indefinitely
        rclpy.spin(node_wrapper.node)
    finally:
        # Destroy the movement node when finished
        node_wrapper.destroy_node()
        # Shutdown rclpy
        rclpy.shutdown()
