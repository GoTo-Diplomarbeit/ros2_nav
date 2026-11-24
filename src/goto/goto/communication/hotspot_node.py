from rclpy.service import SrvTypeRequest, SrvTypeResponse
from std_srvs.srv import SetBool

from goto.communication import hotspot_backend
from goto.node_utils import node_runner
from goto.node_utils.node_wrapper import NodeWrapper


class HotspotNode(NodeWrapper):
    def __init__(self):
        super().__init__("hotspot_node", self._setup_channels)

    def _setup_channels(self) -> None:
        self.node.create_service(SetBool, "hotspot/enable", self._set_hotspot_state)
        self.node.get_logger().info("Hotspot service ready")

        # TODO (distance, later): Create the Float32 publisher and the timer here.

    def _set_hotspot_state(self, hotspot_state: SrvTypeRequest, response: SrvTypeResponse) -> SrvTypeResponse:
        # Check if hotspot already in requested state
        if hotspot_state.data == hotspot_backend.hotspot_active():
            response.success = True
            response.message = "ok"

        try:
            # Enable on true, disable on false
            if hotspot_state.data:
                hotspot_backend.hotspot_start()
            else:
                hotspot_backend.hotspot_stop()

            # Return ok
            response.success = True
            response.message = "ok"
        except Exception as e:
            # Log on error
            self.node.get_logger().error(f"Hotspot error: {e}")

            # Return the error message
            response.success = False
            response.message = str(e)
        return response

    def destroy_node(self) -> None:
        # Disable the hotspot when node gets destroyed
        hotspot_response: SetBool = SetBool()
        self._set_hotspot_state(hotspot_state=False, response=hotspot_response)

        # Destroy the node
        self.node.destroy_node()


def main():
    node_runner.run_node(lambda: HotspotNode())


if __name__ == "__main__":
    main()
