import asyncio
import threading

import uvicorn
from fastapi import FastAPI
from fastapi.responses import StreamingResponse

from go2_nav.communication.camera_node import CameraNode
from go2_nav.node_utils import node_runner


class CameraHost:
    """
    Creates the Camera and serves its latest frames as an MJPEG stream on /video.
    """

    def __init__(self) -> None:
        self.camera_node = CameraNode()
        self._application = FastAPI()
        self._application.add_api_route("/video", self._video_stream)

    async def _generate_frames(self):
        while True:
            if self.camera_node.latest_jpeg_bytes:
                yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + self.camera_node.latest_jpeg_bytes + b"\r\n"
            await asyncio.sleep(0.03)

    async def _video_stream(self) -> StreamingResponse:
        return StreamingResponse(self._generate_frames(), media_type="multipart/x-mixed-replace; boundary=frame")

    def run(self) -> None:
        uvicorn.run(self._application, host="0.0.0.0", port=8000)


def create_camera_node() -> CameraNode:
    camera_host = CameraHost()
    threading.Thread(target=camera_host.run, daemon=True).start()
    return camera_host.camera_node


def main() -> None:
    node_runner.run_node(create_camera_node)


if __name__ == '__main__':
    main()
