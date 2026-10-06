from typing import Callable, AsyncGenerator

import uvicorn
from fastapi import FastAPI
from fastapi.responses import HTMLResponse
from starlette.responses import StreamingResponse

_PAGE = """
<body style="margin:0;background:#000">
    <img src="/video" style="width:100vw;height:100vh;object-fit:contain">
</body>
"""


class PhoneServer:
    def __init__(self, generate_frames: Callable[[], AsyncGenerator[bytes, None]]):
        self._generate_frames = generate_frames
        self._application = FastAPI()
        self._application.add_api_route("/", self._page)
        self._application.add_api_route("/video", self._video_stream)

    async def _page(self) -> HTMLResponse:
        return HTMLResponse(_PAGE)

    async def _video_stream(self) -> StreamingResponse:
        return StreamingResponse(
            self._generate_frames(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    def run(self) -> None:
        uvicorn.run(self._application, host="0.0.0.0", port=8000)
