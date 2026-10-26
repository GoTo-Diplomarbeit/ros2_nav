"""
Camera routes and the app websocket (old VideoServer).
"""
import asyncio

import cv2
import numpy as np
from fastapi import Request
from sensor_msgs.msg import CompressedImage
from starlette.responses import Response, StreamingResponse

from go2_nav.communication.server.web_server import RouteWrapper, WebsocketWrapper


class FrontCamera:
    """
    Holds the newest front camera image.
    """

    def __init__(self) -> None:
        self.snapshot_bytes: bytes | None = None

    def update_image(self, compressed_image: CompressedImage) -> None:
        """
        Sets the newest image that gets served.
        """
        frame = cv2.imdecode(np.frombuffer(compressed_image.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        frame = cv2.resize(frame, (480, 270))  # smaller resolution
        ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 10])  # lower quality
        if ok:
            self.snapshot_bytes = jpg.tobytes()


class FrontCameraRoute(RouteWrapper):
    """
    Serves the newest image from the front camera.
    """

    def __init__(self, camera: FrontCamera) -> None:
        super().__init__("/front_camera", ["GET"])
        self._camera = camera

    async def on_request(self, request: Request) -> Response:
        if self._camera.snapshot_bytes is None:
            return Response(status_code=404, content="No front camera snapshot")
        return Response(status_code=200, content=self._camera.snapshot_bytes, media_type="image/jpeg")


class StreamRoute(RouteWrapper):
    """
    Serves the front camera as an mjpeg stream.
    """

    def __init__(self, camera: FrontCamera, path: str = "/stream.mjpg") -> None:
        super().__init__(path, ["GET"])
        self._camera = camera

    async def on_request(self, request: Request) -> Response:
        async def frames():
            while True:
                if self._camera.snapshot_bytes:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                           + self._camera.snapshot_bytes + b"\r\n")
                await asyncio.sleep(0.05)

        return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")


class AppWebsocket(WebsocketWrapper):
    """
    Websocket between app and robot. Set on_command to receive the app's messages.
    """

    def __init__(self) -> None:
        super().__init__("/ws")

        # Function that gets called with every text message from the app
        self.on_command = None

    def on_websocket_connected(self) -> str | None:
        return None

    def on_websocket_receive(self, received_data: str) -> str | None:
        if self.on_command:
            self.on_command(received_data)
        return None

    def on_websocket_disconnect(self) -> None:
        pass
