""" Video Server for android app """
import asyncio
import threading

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI
from sensor_msgs.msg import CompressedImage
from starlette.responses import Response, StreamingResponse


class VideoServer:
    """
    Streams the front camera on port 8000 of the robot.
    """

    # -- Private

    def __init__(self):
        # Create a new api for video streaming
        self._application = FastAPI()

        # Add the required routes
        self._add_routes()

        # The image that gets served on port 8000
        self._front_camera_snapshot_bytes = None

    def _add_routes(self) -> None:
        """
        Adds all the routes to FastAPI
        Returns: None

        """
        self._application.add_api_route("/front_camera", self._serve_front_camera_snapshot)

        self._application.add_api_route("/stream.mjpg", self._serve_stream)

    async def _serve_front_camera_snapshot(self) -> Response:
        """
        Serves the newest image from the front camera.
        Returns: None

        """
        if self._front_camera_snapshot_bytes is None:
            return Response(status_code=404, content="No front camera snapshot")

        return Response(status_code=200, content=self._front_camera_snapshot_bytes, media_type="image/jpeg")

    async def _serve_stream(self):
        async def frames():
            while True:
                if self._front_camera_snapshot_bytes:
                    yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                           + self._front_camera_snapshot_bytes + b"\r\n")
                await asyncio.sleep(0.05)

        return StreamingResponse(frames(), media_type="multipart/x-mixed-replace; boundary=frame")

    # -- Public

    def start(self) -> None:
        """
        Creates a server and starts it on port 8000.
        Returns: None

        """
        # Create the configuration for uvicorn
        uvicorn_config = uvicorn.Config(self._application, host="0.0.0.0", port=8000)

        # Run application via uvicorn on: ipaddress:8000
        video_server = uvicorn.Server(uvicorn_config)

        # Starts the video server on another thread
        threading.Thread(target=video_server.run, daemon=True).start()

    def update_image(self, compressed_image: CompressedImage):
        """
        Sets the compressed_image to the newest image that gets served on port 8000.
        Args:
            compressed_image: The image that should be served on port 8000.
        """
        current_frame = cv2.imdecode(np.frombuffer(compressed_image.data, np.uint8), cv2.IMREAD_COLOR)
        if current_frame is None:
            return
        current_frame = cv2.resize(current_frame, (480, 270))  # smaller resolution
        ok, jpg = cv2.imencode(".jpg", current_frame, [cv2.IMWRITE_JPEG_QUALITY, 10])  # lower quality
        if ok:
            self._front_camera_snapshot_bytes = jpg.tobytes()
