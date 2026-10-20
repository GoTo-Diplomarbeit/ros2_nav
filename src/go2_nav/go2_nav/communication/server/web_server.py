"""
Holds everything needed for the WebServer class.
"""
import threading
from abc import ABC, abstractmethod
from urllib.request import Request

import uvicorn
from fastapi import FastAPI
from starlette.websockets import WebSocket, WebSocketDisconnect


class WebServer:
    """
    The webserver class used for android device communication.
    """

    def __init__(self) -> None:
        self._application = FastAPI()

    def start(self) -> None:
        """
        Creates a server and starts it on port 8000.
        Returns: None

        """
        # Create the configuration for uvicorn - Use the ip of the device and port 8000 for the server.
        uvicorn_config = uvicorn.Config(self._application, host="0.0.0.0", port=8000)

        # Run application via uvicorn on ipaddress:8000
        video_server = uvicorn.Server(uvicorn_config)

        # Starts the video server on another thread
        threading.Thread(target=video_server.run, daemon=True).start()

    def register_route(self, route_wrapper: "RouteWrapper") -> None:
        """
        Registers a route with the defined endpoint and the defined http requests.
        Calls the function on_request when an http request is received.
        Args:
            route_wrapper: The route wrapper to register.

        Returns: None

        """

        @self._application.api_route(route_wrapper.route_endpoint, methods=route_wrapper.http_methods)
        async def route_handler(request: Request) -> None:
            """
            Internal function that handles the http requests.
            Args:
                request: Internal http request object,

            Returns: None

            """
            return await route_wrapper.on_request(request)

    async def register_websocket(self, websocket_wrapper: "WebsocketWrapper") -> None:
        """
        Registers a websocket with the defined endpoint and
        calls the functions on_websocket_receive and on_websocket_disconnect from WebsocketWrapper.

        Args:
            websocket_wrapper: The websocket wrapper to register.
        """

        @self._application.websocket(websocket_wrapper.websocket_endpoint)
        async def websocket_handler(websocket: WebSocket) -> None:
            """
            Internal function that handles the websocket connection.
            Args:
                websocket: Internal websocket object.
            """
            await websocket.accept()
            try:
                # Call the callback function when the websocket connects
                websocket_wrapper.on_websocket_connected()

                while True:
                    # Await a message from the websocket
                    received_message = await websocket.receive_text()

                    # Call the callback function when the websocket receives a message
                    websocket_wrapper.on_websocket_receive(received_message)
            except WebSocketDisconnect:
                # Call the callback function when the websocket disconnects
                websocket_wrapper.on_websocket_disconnect()


class RouteWrapper(ABC):
    """
    Route wrapper for the WebServer class.
    """

    def __init__(self, route_endpoint: str, methods: list[str]) -> None:
        self.route_endpoint = route_endpoint
        self.http_methods = methods or ["Get"]

    @abstractmethod
    async def on_request(self, request: Request) -> None:
        """
        Gets called when the route receives an http request.
        """
        pass


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

    @abstractmethod
    def on_websocket_connected(self) -> str | None:
        """
        Callback that gets called when the websocket connects.
        Returns: str | None - Returns an optional message to the device that got connected to the websocket.

        """
        pass

    @abstractmethod
    def on_websocket_receive(self, received_data: str) -> str | None:
        """
        Callback that gets called when the websocket received data. Data gets cast to str.
        Args:
            received_data: The data that was received from the websocket.

        Returns: An optional message to the server as a response. Use it to send a success or error message.
        """
        pass

    @abstractmethod
    def on_websocket_disconnect(self) -> None:
        """
        Callback that gets called when the websocket disconnects.
        """
        pass
