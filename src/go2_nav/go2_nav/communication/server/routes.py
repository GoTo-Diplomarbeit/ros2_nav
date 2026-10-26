from go2_nav.communication.server.video_routes import AppWebsocket, FrontCamera, FrontCameraRoute, StreamRoute
from go2_nav.communication.server.web_server import WebServer

# Shared objects, other nodes can import these
front_camera = FrontCamera()
people_debug_camera = FrontCamera()
app_websocket = AppWebsocket()


def register_routes(web_server: WebServer) -> None:
    web_server.register_route(FrontCameraRoute(front_camera))
    web_server.register_route(StreamRoute(front_camera))
    web_server.register_route(StreamRoute(people_debug_camera, "/people_debug.mjpg"))
    web_server.register_websocket(app_websocket)
