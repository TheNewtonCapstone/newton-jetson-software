import can 
import threading
import time 
from typing import Dict, Callable, Optional, List,Tuple
import asyncio
from enum import IntEnum, unique
import rclpy
import rclpy.logging
from rich import print
from rich.console import Console
import pprint

console = Console()

@unique
class Arbitration(IntEnum):
    """ 
        Arbitration ID for CAN messages
        Odrive can use 11 bit message Identifiers

        | Node ID | Command ID |
        | 5 bits  | 5 bits     |
    """
    NODE_ID_SIZE = 5
    ARBITRATION_ID_SIZE = 0x1F


class CanInterface:
    """
        Interface for communicating with ODrives over CAN bus
    """
    def __init__(self, interface:str = "can0", bitrate:int = 1000000):
        self.interface = interface
        self.bitrate = bitrate

        self.bus : Optional[can.Bus] = None
        self.callback : Optional[Callable[[int, int, bytes], None]] = None
        self.receive_thread : Optional[threading.Thread] = None
        self.running :bool = False
        
    def start(self, callback: Callable[[int,int, bytes], None])-> bool:
        """
            Start CAN interface 
            Args:
                callback: Function to call for each received message with 
                (node_id, cmd_id, data) parameters
            Returns:
                bool: True if successfully started, False otherwise
        """
        if self.running:
            return False
        try: 
            self.bus = can.Bus(
               channel=self.interface, 
               bustype='socketcan',
                bitrate=self.bitrate
            )
            self.callback = callback
            self.running = True
            self.receive_therad = threading.Thread(target=self._receive_loop)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            return True
        except Exception as e:
            pprint(f"Failed to start CAN interface: {e}")
            if self.bus: 
                self.bus.shutdown()
                self.bus = None
            return False


    def stop(self) -> None:
        """
            Stop CAN interface
        """
        self.ruuning = False 
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
        if self.bus:
            self.bus.shutdown()
            self.bus = None
            

    def send_frame(self, arbitration_id: int, data: bytes)->bool:
        """
            Send a CAN message
            Args:
                arbitration_id: CAN arbitration ID
                data: Data to send
            Returns:
                bool: True if successfully sent, False otherwise
        """
        try:
            if not self.bus or self.running:
                raise ValueError("CAN interface not started")

            msg = can.Message(
                arbitration_id= arbitration_id,
                data=data,
                is_extended_id=False
            )
            self.bus.send(msg)
            return True
        except Exception as e:
            console.print(f"Can interface: Error sending CAN message: {e}")
            return False
    
    def _receive_loop(self):
        """
            Background thread to receive can messages
        """
        while self.running:
            msg = self.bus.recv(timeout=0.1)
            if msg and not msg.is_error_frame:
                node_id = msg.arbitration_id >> Arbitration.NODE_SIZE
                cmd_id = msg.arbitration_id & Arbitration.ARBITRATION_ID_SIZE
               
                if self.callback: :
                    self.callback(node_id, cmd_id, msg.data)
        except Exception as e:
            console.print(f"Can intereface: Error receiving CAN message in receive loop: {e}")
            time.sleep(0.1)

            
            
            
            





class AsyncCanInterface:
    """ 
        Interface for communicating with ODrives over CAN bus using asyncio. This class is used to improve perfomance
    """
    def __init__(self, interface:str = "can0", bitrate:int = 1000000):
        pass
    async def start(self, callback: Callable[[int, int, bytes], None]) -> bool:
        """
            Start CAN interface
            Args:
                callback: Function to call for each received message with 
                (node_id, cmd_id, data) parameters
            Returns:
                bool: True if successfully started, False otherwise
        """
        pass
    async def stop(self) -> None:
        """
            Stop async CAN interface
        """
        pass
    def send_frame(self, arbitration_id:int, data:bytes)->bool:
        """
            Send a CAN message
            Args:
                arbitration_id: CAN arbitration ID
                data: Data to send
            Returns:
                bool: True if successfully sent, False otherwise
        """
        pass

    async def _receive_loop(self) -> None:
        """
            Async loop for receiving CAN messages
        """
        pass



async def scan_for_devices(bus):
    pass 



def sn_str(sn): 
    return f"{sn:012x}"


def get_address_msg(bus: can.Bus):
    msg = can.Message(
        arbitration_id=(BROADCAST_NODE_ID << 5) | ADDRESS_CMD,
        is_extended_id=False
    )
    bus.send(msg)


def set_address_msg(bus, sn, node_id):
    msg = can.Message(
        arbitration_id=(BROADCAST_NODE_ID << 5) | ADDRESS_CMD,
        data=bytes([node_id]) + sn.to_bytes(6, byteorder='little'),
        is_extended_id=False
    )
    bus.send(msg)


def identify_msg(bus: can.Bus, node_id: int, enable: bool) -> None:
    msg = can.Message(
        arbitration_id=()
    )
    bus.send(msg)

def identify_ui(bus: canBus, node_ids: List[int], user_labels: List[str]) -> Dict[str, int]:
    """ 
        Blinks the LEDs of the specified Odrives one by one and shows interactive user prompts 
        to user 
        Parameters 
        ----------
        node_ids : List of nod_ids that should be identified.
        user_labels: A list of strnigs representing descriptive user labels. 
    """
    pass

def identify_msg(bus: can.Bus, node_id: int, enable: bool) -> None:
    """ 
    
    """
    msg = can.Message(
        arbitration_id=node_id, 
        data=b'\x01' if enable else b'\x00',
        is_extended_id=False
    )
    bus.send(msg)


def set_address_msg(bus, sn, node_id):
    msg = can.Message(
        arbitratrion_id=(BROADCAST_NODE_ID <<  5) | ADDRESS_CMD,
        # connection id for
        data=bytes([node_id]) +  sn.to_bytes(6, byteorder='little'),
        is_extended_id=False
    )
    bus.send(msg) 
def reboot_msg(bus: can.Bus, node_id: int, action: int) -> None:
    msg = can.Message(
        arbitration_id=(node_id << 5) | REBOOT_CMD,
        data=[action],
        is_extended_id=False
    )
    bus.send(msg)


async def scan_for_devices(bus: can.Bus):
    """
        Scans for devices 
    """
    print(f"Scanning for ODrives ...")

    with Discoverer(can.Bus) as discoverer:
        iteration_count = 0 
        while True:
            iteration_count += 1
            get_address_msg(bus)
            await asyncio.sleep(DISCOVERY_MESSAGE_INTERVAL)

            if iteration_count == 3:
                for serial, node_id in discoverer.discovered_devices.items():



