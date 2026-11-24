from std_srvs.srv import SetBool

from goto.communication import hotspot_backend
from goto.node_utils import node_runner
from goto.node_utils.node_wrapper import NodeWrapper

# TODO (distance, later): Re-add "from std_msgs.msg import Float32" once the
#   distance measurement is implemented.


class HotspotNode(NodeWrapper):
    def __init__(self):
        super().__init__("hotspot_node", self._setup_channels)

    def _setup_channels(self) -> None:
        self.node.create_service(SetBool, "hotspot/enable", self._on_enable)
        self.node.get_logger().info("Hotspot service ready")
        # TODO (distance, later): Create the Float32 publisher and the timer here.

    def _on_enable(self, request, response):
        # TODO (optional): Check whether the hotspot is already in the requested
        #   state (hotspot_backend.hotspot_active()) and do nothing if so.
        # TODO (optional): Only allow switching in certain mission states
        #   (self.is_active()) if required. Currently the service works in every state.
        try:
            if request.data:
                hotspot_backend.hotspot_start()
            else:
                hotspot_backend.hotspot_stop()
            response.success = True
            response.message = "ok"
        except Exception as e:
            self.node.get_logger().error(f"Hotspot error: {e}")
            response.success = False
            response.message = str(e)
        return response

    # TODO (optional): Override destroy_node() and call hotspot_stop() there if the
    #   hotspot should be switched off when the node shuts down.


def main():
    node_runner.run_node(lambda: HotspotNode())


if __name__ == "__main__":
    main()
