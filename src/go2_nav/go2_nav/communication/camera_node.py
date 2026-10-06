"""
Camera Node
"""
import threading

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from go2_nav.communication.camera_host import CameraHost
from go2_nav.communication.server import PhoneServer
from go2_nav.node_utils import node_runner
from go2_nav.node_utils.node_wrapper import NodeWrapper


class CameraNode(NodeWrapper):
    """ Camera Node """

    def __init__(self):
        super().__init__("camera", self._setup_channels)

        self.latest_jpeg_bytes = None

    def _setup_channels(self, *args, **kwargs) -> None:
        self.compressed_image_subscription = self.node.create_subscription(
            CompressedImage,
            "/rgb_image/compressed",
            self._store_image,
            qos_profile_sensor_data,
        )

    def _store_image(self, image_message: CompressedImage) -> None:
        self.latest_jpeg_bytes = bytes(image_message.data)


def create_camera_node() -> CameraNode:
    camera_node = CameraNode()
    camera_host = CameraHost(camera_node)
    phone_server = PhoneServer(camera_host.generate_frames)
    threading.Thread(target=phone_server.run, daemon=True).start()
    return camera_node


def main() -> None:
    node_runner.run_node(create_camera_node)


if __name__ == '__main__':
    main()
