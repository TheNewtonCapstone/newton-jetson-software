#!/usr/bin/env python3
"""
Simple ODrive device enumeration script

This script discovers ODrive devices on the CAN bus by 
requesting heartbeat messages from all possible node IDs.
"""

import time
import can
from typing import List
import struct
import argparse

# Constants
NODE_ID_SIZE = 5
MAX_NODE_ID = 0x3E  # Maximum node ID for ODrive CAN protocol
HEARTBEAT_CMD = 0x01  # Heartbeat command ID for ODrive

def make_arbitration_id(node_id: int, cmd_id: int) -> int:
    """
    Create the arbitration ID for a CAN message.
    
    Args:
        node_id: Node ID of the target ODrive
        cmd_id: Command ID of the message
        
    Returns:
        int: Arbitration ID
    """
    return (node_id << NODE_ID_SIZE) | cmd_id

def enumerate_devices(can_interface: str, bitrate: int = 1000000, timeout: float = 3.0) -> List[int]:
    """
    Discover ODrive devices on the CAN bus
    
    Args:
        can_interface: Name of the CAN interface (e.g., "can0")
        bitrate: CAN bus bitrate
        timeout: Time to wait for responses in seconds
        
    Returns:
        List[int]: List of discovered node IDs
    """
    print(f"Initializing CAN interface {can_interface} at {bitrate} bps")
    
    # Open CAN bus
    bus = can.interface.Bus(channel=can_interface, bustype='socketcan', bitrate=bitrate)
    
    # List to store discovered node IDs
    discovered_nodes = []
    
    try:
        # Clear any pending messages
        while bus.recv(timeout=0) is not None:
            pass
        
        print(f"Sending heartbeat requests to all possible node IDs (0-{MAX_NODE_ID})...")
        
        # Send heartbeat request to all possible node IDs
        for node_id in range(MAX_NODE_ID + 1):
            arb_id = make_arbitration_id(node_id, HEARTBEAT_CMD)
            # Send RTR (Remote Transmission Request) frame
            msg = can.Message(arbitration_id=arb_id, is_remote_frame=True, is_extended_id=False)
            bus.send(msg)
            
            # Also send a regular frame with empty data as a fallback
            # Some CAN interfaces might not support RTR frames
            msg = can.Message(arbitration_id=arb_id, data=[], is_extended_id=False)
            bus.send(msg)
        
        # Wait for responses
        print(f"Waiting {timeout} seconds for responses...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            msg = bus.recv(timeout=0.1)
            
            if msg is not None and not msg.is_error_frame:
                # Extract node_id and cmd_id from arbitration ID
                node_id = msg.arbitration_id >> NODE_ID_SIZE
                cmd_id = msg.arbitration_id & ((1 << NODE_ID_SIZE) - 1)
                
                # If this is a heartbeat message and the node is not already discovered
                if cmd_id == HEARTBEAT_CMD and node_id not in discovered_nodes:
                    # Parse axis state if data is available
                    state_name = "Unknown"
                    if len(msg.data) >= 5:
                        state = msg.data[4]
                        state_names = {
                            0: "UNDEFINED",
                            1: "IDLE",
                            8: "CLOSED_LOOP_CONTROL"
                        }
                        state_name = state_names.get(state, f"STATE_{state}")
                    
                    discovered_nodes.append(node_id)
                    print(f"Discovered ODrive with node_id {node_id} (State: {state_name})")
    
    finally:
        # Close the CAN bus
        bus.shutdown()
    
    return discovered_nodes

def main():
    parser = argparse.ArgumentParser(description="Simple ODrive device enumeration via CAN")
    parser.add_argument('--interface', type=str, default='can0', 
                        help='CAN interface to use (default: can0)')
    parser.add_argument('--bitrate', type=int, default=1000000, 
                        help='CAN bitrate (default: 1000000)')
    parser.add_argument('--timeout', type=float, default=3.0, 
                        help='Time to wait for responses in seconds (default: 3.0)')
    args = parser.parse_args()
    
    print("ODrive CAN Device Enumeration Tool")
    print("----------------------------------")
    
    interface="can0"
    bitrate=1000000
    timeout=3.0
    discovered_nodes = enumerate_devices(interface, bitrate, timeout)
    
    if not discovered_nodes:
        print("\nNo ODrive devices found on the CAN bus!")
    else:
        print(f"\nFound {len(discovered_nodes)} ODrive devices with node IDs: {discovered_nodes}")
        print("\nTo use these devices in your code:")
        print("    devices = {")
        for node_id in discovered_nodes:
            print(f"        {node_id}: ODriveDevice({node_id}, can_send_function),")
        print("    }")

if __name__ == "__main__":
    main()