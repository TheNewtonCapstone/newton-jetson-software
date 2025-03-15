import can 
from typing import List, Dict
import asyncio
from .discoverer import Discoverer
from enum import IntEnum, unique

ADDRESS_CMD = 0x06 
REBOOT_CMD = 0x16
CLEAR_ERRORS_CMD = 0x18

BROADCAST_NODE_ID = 0x3f
MAX_NODE_ID = 0x3e

DISCOVERY_MESSAGE_INTERVAL = 0.6 
TIMEOUT =  3.0 

REBOOT_ACTION_REBOOT = 0
REBOOT_ACTION_SAVE = 1 
REBOOT_ACTION_ERASE = 2 
NODE_ID_SIZE = 5

@unique
class Arbitration(IntEnum):
    NODE_SIZE = 5

@unique
class OdriveCANCommands(IntEnum):
    """
        Enum class that contains the ODrive CAN commands. 
        See https://docs.odriverobotics.com/v/latest/manual/can-protocol.html
        for more information.
    """
    GET_VERSION = 0x00
    GET_HEARTBEAT = 0x01
    ESTOP = 0x02
    GET_ERROR = 0x03
    RXS_DO= 0x04
    TXS_DO = 0x05
    GET_ADDRESS = 0x06
    SET_AXIS_STATE = 0x07
    GET_ENCODER_ESTIMATES = 0x09

    SET_CONTROLLER_MODE = 0x00b
    SET_INPUT_POS = 0x0c
    SET_INPUT_VEL = 0x0d
    SET_INPUT_TORQUE = 0x0e
    SET_LIMITS = 0x0f
    SET_ABSOLUTE_POSITION = 0x19

    SET_TRAJ_VEL_LIMIT = 0x11
    SET_TRAJ_ACCEL_LIMIT = 0x12
    SET_TRAJ_INERTIA = 0x13
    
    GET_IQ = 0x14
    GET_TEMPERATURE = 0x15

    REBOOT = 0x16
    GET_BUS_VOLTAGE = 0x17
    CLEAR_ERRORS = 0x18
    SET_POS_GAINS = 0x1a
    SET_VEL_GAINS = 0x1b
    GET_TORQUE = 0x1c
    GET_POWERS = 0x1d
    ENTER_DFU_MODE = 0x1e


@unique
class ControlMode(IntEnum):
    """
        Enum class that contains the control modes of the ODrive. 
    """
    VOLTAGE_CONTROL = 0
    TORQUE_CONTROL = 1
    VELOCITY_CONTROL = 2
    POSITION_CONTROL = 3

@unique
class InputMode(IntEnum):
    """
        Enum class that contains the input modes of the ODrive. 
    """
    INACTIVE = 0
    PASSTHROUGH = 1
    VEL_RAMP = 2
    POS_FILTER = 3
    MIX_CHANNELS = 4
    TRAP_TRAJ = 5


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



