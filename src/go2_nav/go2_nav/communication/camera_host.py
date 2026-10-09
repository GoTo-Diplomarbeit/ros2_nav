import asyncio
from typing import AsyncGenerator, TYPE_CHECKING

if TYPE_CHECKING:
    from go2_nav.communication.camera_node import CameraNode


class CameraHost:
    """ Turns the latest camera JPEG into an MJPEG frame stream. """

    def __init__(self, camera_node: "CameraNode") -> None:
        self._camera_node = camera_node

    async def generate_frames(self) -> AsyncGenerator[bytes, None]:
        """
        Generate a new frame from the camera of the simulator every 30 ms
        """
        while True:
            jpeg_bytes = self._camera_node.latest_jpeg_bytes
            if jpeg_bytes:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg_bytes + b"\r\n"
            await asyncio.sleep(0.03)
