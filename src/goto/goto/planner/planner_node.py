from goto.node_utils import node_runner
from goto.node_utils.node_wrapper import NodeWrapper


class PlannerNode(NodeWrapper):
    def __init__(self):
        super().__init__("planner", self._setup_channels)

    def _setup_channels(self):
        pass


# Main function - Called when executing file
def main():
    # Run the movement node
    node_runner.run_node(lambda: PlannerNode())


if __name__ == '__main__':
    main()
