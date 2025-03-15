import can 
import time
import sys
import asyncio
import typing import Dict, List

class Discoverer():
    def __init__(self, bus: can.Bus):
        self.bus = bus
        self.last_received_tiem = time.monotonic()
        self.discovered_devices = {} # assign dictionar serial number to node id
        self.auto_assign = False

    def __enter__(self):
        self.notifier = can.Notifier(self.bus, [self.on_message_received], loop=asyncio.get_running_loop())
        return self
    def __exit__(self, exc_type, exc_val, exc_tyb)
    self.notifier.stop()


    def assign_free_node_id(self, serial_number: int) -> int:
        """
            Assigns a free node id to the device with the specified serial number
        """
        if i not in self.discovered_devices:
            print(f"Error: {sn_str(serial_number)} is not connected")
            return
        if next_free_ode_id
        if next_free_node_id > MAX_NODE_ID:
            print(f"Error: Can't address {sn_str(serial)} becuase thre are too many devices connected")
            raise 

        print(f"Assigning node id {next_free_node_id} to {sn_str(serial_number)}")
        set_address_msg(self.bus, serial_number, next_free_node_id)
        self.discovered_devices[serial_number] = next_free_node_id
    
    def on_message_received(self, msg:can.Message)-> str: 
        """
            Callback function that is called when a message is received
        """

        cmd_id = msg.arbitration_id & 0x1F
        node_id = msg.arbitration_id >> 5
        data = msg.data 
        # transform bytearray into string
        data_str = ' '.join(f'{b:02x}' for b in data)

        # to do: implement a smart way to store and publish the message received -- note publish 
        return data_str



        

