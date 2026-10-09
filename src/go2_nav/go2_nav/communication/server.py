import asyncio
from typing import Callable, Optional

import cv2
import uvicorn
from fastapi import FastAPI
from starlette.responses import Response, StreamingResponse


def encode_frame(frame) -> bytes:
    small_frame = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_AREA)
    success, encoded_buffer = cv2.imencode(".jpg", small_frame, [cv2.IMWRITE_JPEG_QUALITY, 35])
    return encoded_buffer.tobytes()


class VideoServer:
    def __init__(self, get_jpeg: Callable[[], Optional[bytes]]):
        self._get_jpeg = get_jpeg
        self._application = FastAPI()
        self._application.add_api_route("/snapshot.jpg", self._snapshot)
        self._application.add_api_route("/stream.mjpg", self._stream)

    async def _snapshot(self) -> Response:
        jpeg = self._get_jpeg()
        if jpeg is None:
            return Response(status_code=503)
        return Response(jpeg, media_type="image/jpeg")

    async def _stream(self) -> StreamingResponse:
        async def frame_generator():
            previous_jpeg = None
            while True:
                jpeg = self._get_jpeg()
                if jpeg is not None and jpeg is not previous_jpeg:
                    previous_jpeg = jpeg
                    yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + jpeg + b"\r\n"
                await asyncio.sleep(1 / 100)

        return StreamingResponse(frame_generator(), media_type="multipart/x-mixed-replace; boundary=frame")

    def run(self) -> None:
        uvicorn.run(self._application, host="0.0.0.0", port=8000)
