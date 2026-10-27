"""
Holds everything needed for the WebServer class.
"""
import asyncio
import threading
from abc import ABC, abstractmethod

import uvicorn
from fastapi import FastAPI, Request
from starlette.responses import Response
from starlette.websockets import WebSocket, WebSocketDisconnect


class WebServer:
    """
    The webserver class used for android device communication.
    """

    def __init__(self, port: int = 8000) -> None:
        self._port = port
        self._application = FastAPI()

    def start(self) -> None:
        """
        Creates a server and starts it on its own thread.
        """
        uvicorn_config = uvicorn.Config(self._application, host="0.0.0.0", port=self._port)
        server = uvicorn.Server(uvicorn_config)
        threading.Thread(target=server.run, daemon=True).start()

    def register_route(self, route_wrapper: "RouteWrapper") -> None:
        """
        Registers a route and calls on_request for every http request.
        """

        @self._application.api_route(route_wrapper.route_endpoint, methods=route_wrapper.http_methods)
        async def route_handler(request: Request) -> Response:
            return await route_wrapper.on_request(request)

    def register_websocket(self, websocket_wrapper: "WebsocketWrapper") -> None:
        """
        Registers a websocket and calls the callbacks of the wrapper.
        """

        @self._application.websocket(websocket_wrapper.websocket_endpoint)
        async def websocket_handler(websocket: WebSocket) -> None:
            await websocket.accept()

            # Needed so send_to_app works from other threads
            websocket_wrapper.attach(websocket, asyncio.get_running_loop())

            try:
                answer = websocket_wrapper.on_websocket_connected()
                if answer:
                    await websocket.send_text(answer)

                while True:
                    received_message = await websocket.receive_text()
                    answer = websocket_wrapper.on_websocket_receive(received_message)
                    if answer:
                        await websocket.send_text(answer)
            except WebSocketDisconnect:
                websocket_wrapper.detach()
                websocket_wrapper.on_websocket_disconnect()


class RouteWrapper(ABC):
    """
    Route wrapper for the WebServer class.
    """

    def __init__(self, route_endpoint: str, methods: list[str] | None = None) -> None:
        self.route_endpoint = route_endpoint
        self.http_methods = methods or ["GET"]

    @abstractmethod
    async def on_request(self, request: Request) -> Response:
        """
        Gets called when the route receives an http request.
        """


class WebsocketWrapper(ABC):
    """
    Websocket wrapper for the WebServer class.
    """

    def __init__(self, websocket_endpoint: str) -> None:
        """
        Args:
            websocket_endpoint: The endpoint of the websocket. e.g. /ws, /ws/control
        """
        self.websocket_endpoint = websocket_endpoint
        self._client: WebSocket | None = None
        self._loop: asyncio.AbstractEventLoop | None = None

    def attach(self, websocket: WebSocket, loop: asyncio.AbstractEventLoop) -> None:
        """Called by the WebServer when a client connects."""
        self._client = websocket
        self._loop = loop

    def detach(self) -> None:
        """Called by the WebServer when the client disconnects."""
        self._client = None

    def send_to_app(self, text: str) -> None:
        """
        Sends a text message to the connected app. Can be called from any thread.
        """
        if self._client and self._loop:
            asyncio.run_coroutine_threadsafe(self._client.send_text(text), self._loop)

    @abstractmethod
    def on_websocket_connected(self) -> str | None:
        """Called when the websocket connects. Optional message to send back."""

    @abstractmethod
    def on_websocket_receive(self, received_data: str) -> str | None:
        """Called when text is received. Optional message to send back."""

    @abstractmethod
    def on_websocket_disconnect(self) -> None:
        """Called when the websocket disconnects."""
