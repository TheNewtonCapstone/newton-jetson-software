import time
from typing import Dict, List, Optional, Callable
from rich.console import Console

from .device import ODriveDevice, AxisState, ControlMode, InputMode,OdriveCANCommands
from .can_interface import CanInterface, Arbitration

console = Console()

class ODriveManager:
    """
    Manager class for handling multiple ODrive devices on a CAN bus
    """
    def __init__(self, can_interface):
        """
        Args:
            can_interface: CAN interface for communication
        """
        self.can_interface = can_interface
        self.devices: Dict[int, ODriveDevice] = {}  # node_id -> device
        
    def start(self) -> None:
        """
        Start the manager and all devices
        """
        self.can_interface.start(self.process_can_message)
    def add_device(self, node_id: int) -> ODriveDevice:
        if node_id in self.devices:
            console.print(f"[yellow]Warning: Device with node_id {node_id} already exists. Returning existing device.[/yellow]")
            return self.devices[node_id]
        
        device = ODriveDevice(node_id, self.send_can_frame)
        self.devices[node_id] = device
        return device
    
    def remove_device(self, node_id: int) -> bool:
        if node_id in self.devices:
            del self.devices[node_id]
            return True
        return False
    
    def get_device(self, node_id: int) -> Optional[ODriveDevice]:
        return self.devices.get(node_id)
    
    def send_can_frame(self, arbitration_id: int, data: bytes) -> bool:
        """
        Send a CAN frame
        
        Args:
            arbitration_id: CAN arbitration ID
            data: Data to send
            
        Returns:
            bool: True if successful, False otherwise
        """
        return self.can_interface.send_frame(arbitration_id, data)
    
    def process_can_message(self,node_id: int, cmd_id: int, data: bytearray) -> None:
       
        """
        Process an incoming CAN message
        
        Args:
            node_id: Node ID of the sender
            cmd_id: Command ID of the message
            data: Message data
        """
        device = self.get_device(node_id)
        if device:
            device.process_can_message(cmd_id, data)
        else:
            console.print(f"[dim]Received message from unknown device: node_id={node_id}, cmd_id={cmd_id}[/dim]")
            pass
    
    def stop(self) -> None:
        """
        Stop the manager and all devices
        """
        # Optionally send estop to all devices
        for device in self.devices.values():
            device.estop()
        
        # Stop the CAN interface
        self.can_interface.stop()
    
    def enumerate_devices(self, timeout: float = 3.0) -> List[int]:
        """
        Attempt to discover ODrive devices on the bus by requesting heartbeat
        messages from all possible node IDs
        
        Args:
            timeout: Time to wait for responses in seconds
            
        Returns:
            List[int]: List of discovered node IDs
        """
        # Maximum node ID in ODrive CAN protocol is 0x3E (62)
        max_node_id = 0x3E
        discovered_nodes = []
        
        # Create temporary callback to record responses
        original_callback = self.can_interface.callback
        
        def discovery_callback(node_id: int, cmd_id: int, data: bytes) -> None:
            if cmd_id == OdriveCANCommands.GET_HEARTBEAT:
                if node_id not in discovered_nodes:
                    discovered_nodes.append(node_id)
                    console.print(f"[green]Discovered ODrive with node_id {node_id}[/green]")
        
        try:
            # Set discovery callback
            self.can_interface.callback = discovery_callback
            
            # Request heartbeat from all possible node IDs
            console.print("[blue]Enumerating ODrive devices...[/blue]")
            for node_id in range(max_node_id + 1):
                arb_id = (node_id << Arbitration.NODE_ID_SIZE) | OdriveCANCommands.GET_HEARTBEAT
                self.can_interface.send_frame(arb_id, b'')
            
            # Wait for responses
            start_time = time.time()
            while time.time() - start_time < timeout:
                time.sleep(0.1)
            
        finally:
            # Restore original callback
            self.can_interface.callback = original_callback
        
        return discovered_nodes
    
    def initialize_all(self, 
                      control_mode: ControlMode = ControlMode.POSITION_CONTROL,
                      input_mode: InputMode = InputMode.PASSTHROUGH,
                      closed_loop: bool = True) -> None:
        """
        Initialize all devices with common settings
        
        Args:
            control_mode: Control mode to set
            input_mode: Input mode to set
            closed_loop: Whether to enter closed loop control
        """
        for node_id, device in self.devices.items():
            # Set control mode
            if not device.set_controller_mode(control_mode, input_mode):
                console.print(f"[red]Failed to set controller mode for device {node_id}[/red]")
                continue
            
            # Enter closed loop control if requested
            if closed_loop:
                if not device.set_axis_state(AxisState.CLOSED_LOOP_CONTROL):
                    console.print(f"[red]Failed to enter closed loop control for device {node_id}[/red]")
                    continue
            
            console.print(f"[green]Initialized device {node_id}[/green]")
    
    def estop_all(self) -> None:
        """
        Emergency stop all devices
        """
        for node_id, device in self.devices.items():
            if not device.estop():
                console.print(f"[red]Failed to estop device {node_id}[/red]")
            else:
                console.print(f"[green]E-stopped device {node_id}[/green]")