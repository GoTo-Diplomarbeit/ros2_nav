from abc import ABC, abstractmethod
from typing import Callable

import rclpy


class NodeWrapper(ABC):
    """
    Wrapper for each node. Used for node_runner.py.
    """

    def __init__(self, node_name: str, setup_channels: Callable[[], None]) -> None:
        # Node that will get created for every inheritor
        self.node = rclpy.create_node(node_name)

        # Setup all required channels
        setup_channels()

    def destroy_node(self) -> None:
        """
        Function to destroy the node from the class that extends NodeWrapper.
        Returns: None

        """
        self.node.destroy_node()

    @abstractmethod
    def _setup_channels(self, *args, **kwargs):
        ...
