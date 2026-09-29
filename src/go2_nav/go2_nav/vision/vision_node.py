from go2_nav.node_utils import node_runner
from go2_nav.node_utils.node_wrapper import NodeWrapper


class VisionNode(NodeWrapper):
    def __init__(self):
        super().__init__("vision")


# Main function - Called when executing file
def main():
    # Run the movement node
    node_runner.run_node(lambda: VisionNode())


if __name__ == '__main__':
    main()
