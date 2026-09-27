from abc import ABC, abstractmethod

from rclpy.node import Node


class NodeWrapper(ABC):
    """
    Wrapper for each node. Used for node_runner.py.
    """

    @property
    @abstractmethod
    def node(self) -> Node:
        """
        Property to get the node from the class that extends NodeWrapper.
        Returns: The node property of the class

        """
        ...

    @abstractmethod
    def destroy_node(self) -> None:
        """
        Function to destroy the node from the class that extends NodeWrapper.
        Returns: None

        """
        ...
