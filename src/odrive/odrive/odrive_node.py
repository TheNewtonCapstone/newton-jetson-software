import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ODriveNode(Node):
    def __init__(self):
        """Initialize the ODrive CAN controller node"""

        super().__init__("odrive_node")
        ## Declare parameters
        ## get parameters
        # validate paarametes
        # configure devices
        # create pub
        # create sub
        # setup timers
        # start can interface

    def position_callback(self, msg):
        # command for position command messages
        pass

    def shutdown_callback(self):
        # shutdown callback
        pass


def main():
    pass


if __name__ == "__main__":
    main()
