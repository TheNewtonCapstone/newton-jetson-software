import rclpy
import os
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rich.console import Console
from .device_manager import ODriveManager
from .can_interface import CanInterface

class ODriveNode(Node):
    def __init__(self):
        """Initialize the ODrive CAN controller node"""

        super().__init__("odrive_node")
        self.console = Console()
        self.console.print("Initializing ODrive CAN controller node")
        self.can_interface = CanInterface()
        self.manager = ODriveManager(self.can_interface)
        config_file_path = "../../configs/newton.yaml"
        print(f"current working directory: {os.getcwd()}")
        self.manager.start(config_file_path)
        self.manager.enumerate_devices()
        # self.manager.initialize_all()
        self.manager.init_one(6)
        # self.manager.init_one(7)
        # self.manager.init_one(8)
        # self.manager.init_one(9)
        # self.manager.init_one(10)
        # self.manager.init_one(11)
        
        # manager.enumerate_devices()
        


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

    def shutdown(self):
        
        pass


def main():
    rclpy.init()
    node = ODriveNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
