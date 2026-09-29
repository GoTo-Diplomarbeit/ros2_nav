from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
import math

from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from typing import Optional


class MovementBackendType(Enum):
    """
    Specifies which type of backend to use.
    """
    GAZEBO = "Gazebo"
    GO2 = "Go2"


@dataclass(frozen=True)
class NetworkInterface:
    """Typed representation of a network interface name for the real robot, e.g. 'eth0', 'enp3s0'."""
    network_interface_name: str

    def __post_init__(self) -> None:
        if not self.network_interface_name or " " in self.network_interface_name:
            raise ValueError(f"Invalid network interface name: {self.network_interface_name!r}")

    def __str__(self) -> str:
        return self.network_interface_name


class MovementBackend(ABC):
    """
    Abstract class for movement backend. Either Simulator or Real Robot backend.
    """

    @abstractmethod
    def send(self, movement_command: Twist) -> None:
        ...

    @abstractmethod
    def get_yaw(self) -> Optional[float]:
        ...

class Go2Backend(MovementBackend):
    """
    Implements movement backend for the real robot on the network.
    """

    def __init__(self, network_interface: NetworkInterface) -> None:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        from unitree_sdk2py.go2.sport.sport_client import SportClient

        # Create the channel with the defined network interface
        ChannelFactoryInitialize(0, str(network_interface))

        # Create a sportClient
        self._sport_client = SportClient()
        # Set the max network timeout
        self._sport_client.SetTimeout(10.0)
        # Initialize the sportClient
        self._sport_client.Init()

    def get_yaw(self) -> Optional[float]:
        # TODO: Implement IMU Robot
        return None

    def send(self, movement_command: Twist) -> None:
        # Translate the Twist command to sportClient Move
        self._sport_client.Move(movement_command.linear.x, movement_command.linear.y, movement_command.angular.z)

class ImuReaderGazebo:
    """
    Reads data from the IMU sensor in the simulator.
    """
    def __init__(self, node: Node):
        self._yaw = None
        node.create_subscription(Imu, "/imu/data", self._imu_callback, qos_profile_sensor_data)
        # Creates subscription to read data from the sensor

    def _imu_callback(self, imu: Imu):
        q = imu.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._yaw = yaw
        # Calculates the position of the robot

    def get_yaw(self) -> Optional[float]:
        return self._yaw
        # Returns the position

class GazeboBackend(MovementBackend):
    """
    Implements movement backend for the simulator.
    """

    def __init__(self, movement_command_publisher: Publisher, node: Node) -> None:
        """
        Creates a Movement Backend for the gazebo simulator.
        Args:
            movement_command_publisher (Publisher):
        """

        # Save the movement command publisher for the simulator in the class
        self._movement_command_publisher = movement_command_publisher

        self._imu_reader = ImuReaderGazebo(node)

    def get_yaw(self) -> Optional[float]:
        return self._imu_reader.get_yaw()
    # Get the angle and position of the robot in the simulator

    def send(self, movement_command: Twist) -> None:
        # Forward the movement_command to the simulator
        self._movement_command_publisher.publish(movement_command)
