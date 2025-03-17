import rclpy
from rclpy.executors import MultiThreadedExecutor
import os
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rich.console import Console
from .device_manager import ODriveManager
from .can_interface import CanInterface, AsyncCanInterface
import asyncio

class ODriveNode(Node):
    def __init__(self):
        """Initialize the ODrive CAN controller node"""

        super().__init__("odrive_node")
        self.console = Console()
        self.console.print("Initializing ODrive CAN controller node")
        self.can_interface = CanInterface()
        self.manager = ODriveManager(self.can_interface)
        config_file_path = "../../configs/newton.yaml"
        self.manager.load_configs_from_file(config_file_path)
        self.can_interface.start(self.manager.process_can_message)
        self.manager.calibrate_all()

        
        


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
        self.manager.estop_all()
        
        pass


def main():
    rclpy.init()
    node = ODriveNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except ExternalShutdownException:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
