import json
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

from goto.communication.server.routes import app_websocket, front_camera, register_routes
from goto.communication.server.web_server import WebServer
from goto.node_utils import node_runner
from goto.node_utils.node_wrapper import NodeWrapper

CAMERA_TOPIC = "/rgb_image/compressed"

# Speed limits, so a bug in the app can never send a huge speed
MAX_LINEAR = 0.5  # m/s
MAX_ANGULAR = 1.0  # rad/s

# Seconds without a command before the robot stops by itself
TIMEOUT = 0.5


class WebServerNode(NodeWrapper):
    def __init__(self):
        super().__init__("web_server_node", self._setup_channels)

        # Create a webserver with default port 8000
        web_server = WebServer()

        # Register all routes
        register_routes(web_server)

        # Start the webserver
        web_server.start()

        # Needed by the watchdog to detect a lost connection
        self.last_cmd_time = 0.0
        self.moving = False
        self.node.create_timer(0.1, self._watchdog)

    def _setup_channels(self, *args, **kwargs):
        # Camera -> front camera route
        self.node.create_subscription(CompressedImage, CAMERA_TOPIC, front_camera.update_image, 1)

        # Publisher for the movement commands of the app
        self.cmd_pub = self.node.create_publisher(Twist, "planner_cmd_vel", 10)

        # Messages from the app end up in handle_command
        app_websocket.on_command = self.handle_command

    def handle_command(self, message: str) -> None:
        """
        Called from the websocket thread for every message from the app.
        Only {"cmd": "move", "x": .., "y": .., "yaw": ..} is understood.
        """
        try:
            data = json.loads(message)
        except json.JSONDecodeError:
            self.node.get_logger().warning(f"Invalid JSON: {message}")
            return

        if data.get("cmd") != "move":
            return

        def clamp(value, limit):
            return max(-limit, min(limit, float(value)))

        twist = Twist()
        twist.linear.x = clamp(data.get("x", 0.0), MAX_LINEAR)
        twist.linear.y = clamp(data.get("y", 0.0), MAX_LINEAR)
        twist.angular.z = clamp(data.get("yaw", 0.0), MAX_ANGULAR)

        self.cmd_pub.publish(twist)

        self.last_cmd_time = time.monotonic()
        self.moving = any([twist.linear.x, twist.linear.y, twist.angular.z])

    def _watchdog(self) -> None:
        """Safety stop: stops the robot if the app disconnects or stops sending."""
        if self.moving and time.monotonic() - self.last_cmd_time > TIMEOUT:
            self.cmd_pub.publish(Twist())
            self.moving = False

    def destroy_node(self) -> None:
        # Never leave the robot moving after the node exits
        self.cmd_pub.publish(Twist())
        super().destroy_node()


# Main function - Called when executing file
def main():
    # Run the webserver node
    node_runner.run_node(lambda: WebServerNode())


if __name__ == "__main__":
    main()
