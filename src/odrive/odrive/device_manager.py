from typing import Dict, Callable, Optional, List

from device import ODriveDevice


class DeviceManage:
    """
    DeviceManager class is responsible for managing multiple ODrives connected to the host.
    """

    def __init__(self, send_can_frame: Callable[[int, bytes], bool]):
        """
        Initialize the DeviceManager.
        Args:
          send_can_frame: Function to send CAN frames
        """
        pass

    def add_device(self, node_id: int) -> ODriveDevice:
        """
        Add an ODrive device to the manager.
        Args:
          node_id: CAN node ID of the ODrive
        Returns:
          ODriveDevice: The newly created device
        """
        pass

    def get_device(self, node_id: int) -> Optional[ODriveDevice]:
        """
        Get an ODrive device by node ID.
        Args:
          node_id: CAN node ID of the ODrive
        Returns:
          ODriveDevice or None: The device if found, None otherwise
        """
        pass

    def get_all_positions(self) -> List[float]:
        """
        Get current positions of all devices.
        Returns:
          List[float]: List of positions in order of node IDs
        """
        pass

    def get_all_velocities(self) -> List[float]:
        """
        Get current velocities of all devices.
        Returns:
          List[float]: List of velocities in order of node IDs
        """
        pass

    def get_all_torques(self) -> List[float]:
        """
        Get current torques of all devices.
        Returns:
          List[float]: List of torques in order of node IDs
        """
        pass

    def set_all_positions(self, positions: List[float]) -> None:
        """
        Set positions for all devices.
        Args:
          positions: List of positions in order of node IDs
        """
        pass

    def set_all_velocities(self, velocities: List[float]) -> None:
        """
        Set velocities for all devices.
        Args:
          velocities: List of velocities in order of node IDs
        """
        pass

    def arm_all_devices(self) -> None:
        """
        Arm all devices.
        """
        pass

    def disarm_all_devices(self) -> None:
        """
        Disarm all devices.
        """
        pass

    def clear_errors_all_devices(self) -> None:
        """ "
        Clear errors on all devices.
        """
        pass

    def check_watchdog(self) -> None:
        """
        Check the watchdog status of all devices.
        """
        pass

    def process_can_message(self, nod_id: int, cmd_id: int, data: bytes) -> None:
        """
        Process incoming CAN message.
        Args:
          node_id: Node ID from the arbitration ID
          cmd_id: Command ID from the arbitration ID
          data: Raw CAN message data
        """
        pass

    def request_feedback(self) -> None:
        """
        Request feedback from all devices.
        """
        pass
