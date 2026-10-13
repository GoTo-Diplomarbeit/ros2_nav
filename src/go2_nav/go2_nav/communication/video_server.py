""" Video Server for android app """
import asyncio
import atexit
import socket
import threading

import cv2
import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from sensor_msgs.msg import CompressedImage
from starlette.responses import Response, StreamingResponse
from zeroconf import NonUniqueNameException, ServiceInfo, Zeroconf


class VideoServer:
    """
    Streams the front camera on port 8000 of the robot.
    """

    # -- Private

    def __init__(self, logger) -> None:
        # Logger of the node the video server belongs to
        self._logger = logger

        # Create a new api for video streaming
        self._application = FastAPI()

        # Add the required routes
        self._add_routes()

        # The image that gets served on port 8000
        self._front_camera_snapshot_bytes = None

        # The app that is currently connected to the websocket (only one at a time)
        self._client = None

        # The event loop of uvicorn, needed to send messages from other threads
        self._loop = None

        # Function that gets called with every text message from the app
        self.on_command = None

    def _add_routes(self) -> None:
        """
        Adds all the routes to FastAPI
        Returns: None

        """
        self._application.add_api_route("/front_camera", self._serve_front_camera_snapshot)

        self._application.add_api_route("/stream.mjpg", self._serve_stream)

        # Websocket for the communication between app and robot
        self._application.add_api_websocket_route("/ws", self._websocket_endpoint)

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

    async def _websocket_endpoint(self, websocket: WebSocket) -> None:
        """
        Handles the websocket connection to the app. Every text message
        that arrives is passed on to on_command.
        Args:
            websocket: The connection of the app.
        """
        await websocket.accept()

        # Remember the loop and the client so send_to_app can use them from other threads
        self._loop = asyncio.get_running_loop()
        self._client = websocket

        try:
            while True:
                message = await websocket.receive_text()
                if self.on_command:
                    self.on_command(message)
        except WebSocketDisconnect:
            # The app closed the connection, nothing to send to anymore
            self._client = None

    def _advertise(self) -> None:
        """Announces the stream on the network via mDNS."""
        mdns_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            mdns_socket.connect(("10.255.255.255", 1))
            ip = mdns_socket.getsockname()[0]
        finally:
            mdns_socket.close()

        self._zeroconf = Zeroconf()
        try:
            self._zeroconf.register_service(ServiceInfo(
                "_mjpeg._tcp.local.",
                "go2-camera._mjpeg._tcp.local.",
                addresses=[socket.inet_aton(ip)],
                port=8000,
                properties={"path": "/stream.mjpg"},
                # Has to be a hostname, otherwise android cannot resolve the service
                server="go2-camera.local.",
            ))
        except NonUniqueNameException:
            # The stream still works, the app just cannot discover it on its own
            self._logger.warning("Could not advertise the camera stream, the service name is already in use")

        atexit.register(self._zeroconf.close)

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

        # Start advertising the stream
        self._advertise()

    def send_to_app(self, text: str) -> None:
        """
        Sends a text message to the connected app. Can be called from any thread.
        Args:
            text: The message that gets sent to the app.
        """
        if self._client and self._loop:
            # Uvicorn runs in its own thread, so the send has to be handed over to its loop
            asyncio.run_coroutine_threadsafe(self._client.send_text(text), self._loop)

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
