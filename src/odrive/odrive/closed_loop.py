import can
import struct

from can_interface import OdriveCANCommands, Arbitration


def main():
    node_id = 0
    bus = can.interface.Bus("can0", bustype="socketcan")

    flush_rx_buffer(bus)

    with can.Bus() as bus:
        for msg in bus:
            print(msg.data)
    bus.send(
        can.Message(
            arbitration_id=(node_id) << Arbitration.NODE_SIZE | 0x07,
            data=struct.pack("B", 0x01),
            is_extended_id=False,
        )
    )

    for msg in bus:
        if msg.arbitration_id == (
            node_id << Arbitration.NODE_SIZE | OdriveCANCommands.GET_HEARTBEAT
        ):
            size = 7
            error, state, result, traj_done = struct.unpack("<BBBB", msg.data[:size])
            if state == 8:
                break
    # set velocity to 1.0 turns/s
    bus.send(
        can.Message(
            arbitration_id=(
                node_id << Arbitration.NODE_SIZE | OdriveCANCommands.SET_INPUT_VEL
            ),
            data=struct.pack("<ff", 1.0, 0.0),
            is_extended_id=False,
        )
    )

    # print encode feedback
    for msg in bus:
        if msg.arbitration_id == (
            node_id << Arbitration.NODE_SIZE | OdriveCANCommands.GET_ENCODER_ESTIMATES
        ):
            pos, vel = struct.unpack("<ff", bytes(msg.data))
            print(f"pos: {pos}, vel: {vel}")


def flush_rx_buffer(bus: can.Bus) -> None:
    """
    Flushes the RX buffer of the CAN bus.
    """
    while bus.recv() is not None:
        pass


if __name__ == "__main__":
    main()
