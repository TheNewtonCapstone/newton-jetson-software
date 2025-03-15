import can 

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



async def scan_for_devices(bus):
    pass 



def sn_str(sn): 
    return f"{sn:012x}"


def get_address_msg(bus: can.Bus):
    msg = can.Message(
        arbitration_id=(BROADCAST_NODE_ID << 5) | ADDRESS_CMD,
        is_extended_id=False
        is_remote_frame=True
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



