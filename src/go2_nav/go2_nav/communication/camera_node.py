"""
Camera Node
"""

from sensor_msgs.msg import CompressedImage

from go2_nav.node_utils.node_wrapper import NodeWrapper


class CameraNode(NodeWrapper):
    """ Camera Node """

    def __init__(self):
        super().__init__("camera", self._setup_channels)

        self.latest_jpeg_bytes = None

    def _setup_channels(self, *args, **kwargs) -> None:
        self.compressed_image_subscription = self.node.create_subscription(CompressedImage,
                                                                           "/rgb_image/compressed",
                                                                           self._store_image, 10)

    def _store_image(self, image_message: CompressedImage) -> None:
        self.latest_jpeg_bytes = bytes(image_message.data)
