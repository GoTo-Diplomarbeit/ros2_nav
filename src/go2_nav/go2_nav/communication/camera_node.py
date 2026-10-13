"""Camera node: serves the camera to the app and turns app commands into movement."""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from go2_nav.communication.video_server import VideoServer

# Speed limits, so a bug in the app can never send a huge speed
MAX_LINEAR = 0.5  # m/s
MAX_ANGULAR = 1.0  # rad/s

# Seconds without a command before the robot stops by itself
TIMEOUT = 0.5


class CameraNode(Node):
    """
    Forwards camera images to the video server and publishes the
    movement commands from the app on planner_cmd_vel.
    """

    def __init__(self) -> None:
        super().__init__("camera_node")

        # Camera -> video server
        self.create_subscription(
            CompressedImage,
            "/rgb_image/compressed",  # adjust to your topic
            self._on_image,
            1,
        )

        # Publisher for the movement commands of the app
        self.cmd_pub = self.create_publisher(Twist, "planner_cmd_vel", 10)

        # Needed by the watchdog to detect a lost connection
        self.last_cmd_time = 0.0
        self.moving = False
        self.create_timer(0.1, self._watchdog)

        # Video + websocket server, app messages end up in handle_command
        self.video_server = VideoServer(self.get_logger())
        self.video_server.on_command = self.handle_command
        self.video_server.start()

    def _on_image(self, msg: CompressedImage) -> None:
        """
        Passes a new camera image on to the video server.
        Args:
            msg: The newest image of the camera.
        """
        self.video_server.update_image(msg)

    def handle_command(self, message: str) -> None:
        """
        Called from the websocket thread for every message from the app.
        Only {"cmd": "move", "x": .., "y": .., "yaw": ..} is understood.
        Args:
            message: The raw JSON text sent by the app.
        """
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            self.get_logger().warning(f"Invalid JSON: {message}")
            return

        if data.get("cmd") != "move":
            return

        def clamp(value, limit):
            """Limits value to the range -limit..limit."""
            return max(-limit, min(limit, float(value)))

        # Build the velocity command from the app values
        twist = Twist()
        twist.linear.x = clamp(data.get("x", 0.0), MAX_LINEAR)
        twist.linear.y = clamp(data.get("y", 0.0), MAX_LINEAR)
        twist.angular.z = clamp(data.get("yaw", 0.0), MAX_ANGULAR)

        self.cmd_pub.publish(twist)

        # Remember when and whether the robot is moving for the watchdog
        self.last_cmd_time = time.monotonic()
        self.moving = any([twist.linear.x, twist.linear.y, twist.angular.z])

    def _watchdog(self) -> None:
        """
        Safety stop: stops the robot if the app disconnects or stops sending.
        Runs every 0.1 seconds.
        """
        if self.moving and time.monotonic() - self.last_cmd_time > TIMEOUT:
            self.cmd_pub.publish(Twist())
            self.moving = False


def main(args=None) -> None:
    """Starts the node and stops the robot when it shuts down."""
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Never leave the robot moving after the node exits
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
