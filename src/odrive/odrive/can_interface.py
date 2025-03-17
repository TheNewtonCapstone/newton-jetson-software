import can
import threading
import time
from typing import Dict, Callable, Optional, List, Tuple
import asyncio
from enum import IntEnum, unique
import rclpy
import rclpy.logging
from rich import print
from rich.console import Console

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

    def __init__(self, interface: str = "can0", bitrate: int = 1000000):
        self.interface = interface
        self.bitrate = bitrate

        self.bus: Optional[can.Bus] = None
        self.callback: Optional[Callable] = None
        self.receive_thread: Optional[threading.Thread] = None
        self.running: bool = False

    def start(self, callback: Callable[[int, int, bytes], None]) -> bool:
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
            print(type(callback))
            self.bus = can.Bus(
                channel=self.interface, bustype="socketcan", bitrate=self.bitrate
            )
            self.callback = callback
            self.running = True
            self.receive_thread = threading.Thread(target=self._receive_loop)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            return True
        except Exception as e:
            console.print(
                f"[red]Can interface: Error starting CAN interface: {e}[/red]"
            )
            if self.bus:
                self.bus.shutdown()
                self.bus = None
            return False

    def stop(self) -> None:
        """
        Stop CAN interface
        """
        self.running = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=1.0)
        if self.bus:
            self.bus.shutdown()
            self.bus = None

    def send_frame(self, arbitration_id: int, data: bytes) -> bool:
        """
        Send a CAN message
        Args:
            arbitration_id: CAN arbitration ID
            data: Data to send
        Returns:
            bool: True if successfully sent, False otherwise
        """
        try:
            # removed the check for running because we might need to send messages before we want to init the loop
            if not self.bus:
                raise ValueError("CAN interface not started")

            msg = can.Message(
                arbitration_id=arbitration_id, data=data, is_extended_id=False
            )
            self.bus.send(msg)
            return True
        except Exception as e:
            console.print(f"[red]Can interface: Error sending CAN message: {e}[/red]")
            return False

    def _receive_loop(self):
        """
        Background thread to receive can messages
        """
        try:
            while self.running:
                msg = self.bus.recv(timeout=0.1)
                if msg and not msg.is_error_frame:
                    node_id = msg.arbitration_id >> Arbitration.NODE_ID_SIZE
                    cmd_id = msg.arbitration_id & Arbitration.ARBITRATION_ID_SIZE
                    if self.callback:
                        self.callback(node_id, cmd_id, msg.data)
        except Exception as e:
            console.print(
                f"Can intereface: Error receiving CAN message in receive loop: {e}"
            )
            time.sleep(0.1)


class AsyncCanInterface:
    """
    Interface for communicating with ODrives over CAN bus using asyncio. This class is used to improve perfomance
    """

    def __init__(self, interface: str = "can0", bitrate: int = 1000000):
        self.interface = interface
        self.bitrate = bitrate
        self.bus: Optional[can.Bus] = None
        self.callback: Optional[Callable] = None
        self.reader: Optional[can.AsyncBufferedReader] = None
        self.notifier: Optional[can.Notifier] = None
        self.running = False
        self.initilized = False

        
    async def start(self, callback: Callable[[int, int, bytes], None]) -> bool:
        """
        Start CAN interface
        Args:
            callback: Function to call for each received message with
            (node_id, cmd_id, data) parameters
        Returns:
            bool: True if successfully started, False otherwise
        """
        console.print("Starting CAN interface")
        if self.running:
            return False
        try:
            self.bus = can.Bus(
                channel=self.interface, bustype="socketcan", bitrate=self.bitrate
            )
            self.callback = callback
            self.reader = can.AsyncBufferedReader()
            loop = asyncio.get_event_loop()
            self.notifier = can.Notifier(self.bus, [self.reader], loop=loop)

            self.running = True
            self.initilized = True
            
            asyncio.create_task(self._receive_loop())
            await asyncio.sleep(0.1)
            
            return True

        except Exception as e:
            console.print(
                f"[red]Async Can interface: Error starting CAN interface: {e}[/red]"
            )
            if self.bus:
                self.bus.shutdown()
                self.bus = None
            return False

    def stop(self) -> None:
        """
        Stop async CAN interface
        """
        self.running = False
        self.initilized = False
        if self.notifier:
            self.notifier.stop()
        if self.bus:
            self.bus.shutdown()
            self.bus = None

    def send_frame(self, arbitration_id: int, data: bytes) -> bool:
        """
        Send a CAN message
        Args:
            arbitration_id: CAN arbitration ID
            data: Data to send
        Returns:
            bool: True if successfully sent, False otherwise
        """
        try:
            if not self.bus or not self.initilized:
                raise ValueError("CAN interface not started or initialized")
            msg = can.Message(
                arbitration_id=arbitration_id, data=data, is_extended_id=False
            )
            self.bus.send(msg)
            return True
        except Exception as e:
            console.print(f"[red]Async Can interface: Error sending CAN message: {e}[/red]")
            return False

    async def _receive_loop(self) -> None:
        """
        Async loop for receiving CAN messages
        """
        try:
            while self.running:
                msg = await self.reader.get_message()
                if msg and not msg.is_error_frame:
                    node_id = msg.arbitration_id >> Arbitration.NODE_ID_SIZE
                    cmd_id = msg.arbitration_id & Arbitration.ARBITRATION_ID_SIZE
                    if self.callback:
                        self.callback(node_id, cmd_id, msg.data)
        except Exception as e:
            console.print(
                f"Async Can interface: Error receiving CAN message in receive loop: {e}"
            )
            await asyncio.sleep(0.1)
            await self._receive_loop()