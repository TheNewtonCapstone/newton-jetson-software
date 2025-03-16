import can
import struct
import time

# Initialize CAN bus
bus = can.interface.Bus(channel="can0", bustype="socketcan", bitrate=1000000)


# sent the command for a range of ids from  0 to 11 
for node_id in range(12):
    print (f"Checking node_id {node_id}")
    arb_id = (node_id << 5) | 0x01
    bus.send(can.Message(arbitration_id=arb_id, is_extended_id=False, is_remote_frame=True))
    
    
    # listen for response with a timeout
    start_time = time.time()
    while time.time() - start_time < 0.5:
        msg = bus.recv()
        if msg.arbitration_id == arb_id:
            print(f"Node {node_id} found")
            break
    else:
        print(f"Node {node_id} not found")

    
    