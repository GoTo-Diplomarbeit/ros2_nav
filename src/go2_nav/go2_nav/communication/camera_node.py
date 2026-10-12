"""
Camera Node
"""

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from go2_nav.communication.video_server import VideoServer
from go2_nav.node_utils import node_runner
from go2_nav.node_utils.node_wrapper import NodeWrapper


class CameraNode(NodeWrapper):
    """ Camera Node """

    def __init__(self):
        super().__init__("camera", self._setup_channels)

        # Creates a video server for the android app to poll
        self._video_server = VideoServer()

        # Start the video server
        self._video_server.start()

    def _setup_channels(self, *args, **kwargs) -> None:
        """
        Sets up the channels used by the camera node.

        Returns: None

        """
        # Creates a subscription for the compressed rgb_image of the main robot camera
        self.compressed_image_subscription = self.node.create_subscription(
            CompressedImage,
            "/rgb_image/compressed",
            self._on_compressed_image_received,
            qos_profile_sensor_data,
        )

    def _on_compressed_image_received(self, compressed_image: CompressedImage) -> None:
        """
        Sets the compressed image to the newest compressed image coming from subscription
        Args:
            compressed_image: The compressed image from the subscription

        Returns: None

        """

        self._video_server.update_image(compressed_image)


def create_camera_node() -> CameraNode:
    """
    Creates a camera node and sets up the video server for the android app
    Returns: The camera node object

    """
    # Creates a camera node
    camera_node = CameraNode()

    # Returns the camera_node object
    return camera_node


def main() -> None:
    """ Main function that runs the camera node """
    # Runs the camera node
    node_runner.run_node(create_camera_node)


if __name__ == '__main__':
    main()
