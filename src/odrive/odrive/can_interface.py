import can 
import threading
import time 
from typing import List, Dict, Callable, Optional, List,Tuple
import asyncio
from enum import IntEnum, unique
import rclpy
import rclpy.logging

class CanInterface:
    """
        Interface for communicating with ODrives over CAN bus
    """
    def __init__(self, interface:str = "can0", bitrate:int = 1000000):
        pass
        
    def start(self, callback: Callable[[int,int, bytes], None])-> bool:
        """
            Start CAN interface 
            Args:
                callback: Function to call for each received message with 
                (node_id, cmd_id, data) parameters
            Returns:
                bool: True if successfully started, False otherwise
        """
        pass

    def stop(self) -> None:
        """
            Stop CAN interface
        """
        pass 

    def send(self, arbitration_id: int, data:bytes)->bool:
        """
            Send a CAN message
            Args:
                arbitration_id: CAN arbitration ID
                data: Data to send
            Returns:
                bool: True if successfully sent, False otherwise
        """
        pass
    
    def _receive_loop(self):
        """
            Background thread to receive can messages
        """
        pass



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



