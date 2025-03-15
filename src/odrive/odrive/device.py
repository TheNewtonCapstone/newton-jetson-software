import struct
from enum import IntEnum, unique
from typing import Callable, Dict, Optional, List, Tuple

ADDRESS_CMD = 0x06
REBOOT_CMD = 0x16
CLEAR_ERRORS_CMD = 0x18

BROADCAST_NODE_ID = 0x3F
MAX_NODE_ID = 0x3E

DISCOVERY_MESSAGE_INTERVAL = 0.6
TIMEOUT = 3.0

REBOOT_ACTION_REBOOT = 0
REBOOT_ACTION_SAVE = 1
REBOOT_ACTION_ERASE = 2
NODE_ID_SIZE = 5


@unique
class Arbitration(IntEnum):
    NODE_SIZE = 5


class AxisState(IntEnum):
    UNDEFINED = 0
    IDLE = 1
    STARTUP_SEQUENCE = 2
    FULL_CALIBRATION_SEQUENCE = 3
    MOTOR_CALIBRATION = 4
    ENCODER_INDEX_SEARCH = 6
    ENCODER_OFFSET_CALIBRATION = 7
    CLOSED_LOOP_CONTROL = 8
    LOCKIN_SPIN = 9
    ENCODER_DIR_FIND = 10
    HOMING = 11
    ENCODER_HALL_PHASE_CALIBRATION = 12
    ENCODER_HALL_POLARITY_CALIBRATION = 13


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
    RXS_DO = 0x04
    TXS_DO = 0x05
    GET_ADDRESS = 0x06
    SET_AXIS_STATE = 0x07
    GET_ENCODER_ESTIMATES = 0x09

    SET_CONTROLLER_MODE = 0x00B
    SET_INPUT_POS = 0x0C
    SET_INPUT_VEL = 0x0D
    SET_INPUT_TORQUE = 0x0E
    SET_LIMITS = 0x0F
    SET_ABSOLUTE_POSITION = 0x19

    SET_TRAJ_VEL_LIMIT = 0x11
    SET_TRAJ_ACCEL_LIMIT = 0x12
    SET_TRAJ_INERTIA = 0x13

    GET_IQ = 0x14
    GET_TEMPERATURE = 0x15

    REBOOT = 0x16
    GET_BUS_VOLTAGE = 0x17
    CLEAR_ERRORS = 0x18
    SET_POS_GAINS = 0x1A
    SET_VEL_GAINS = 0x1B
    GET_TORQUE = 0x1C
    GET_POWERS = 0x1D
    ENTER_DFU_MODE = 0x1E


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


class ODriveDevice:
    """
    This clas represents a single ODrive device on the CAN bus.
    """

    def __init__(self, node_id: int, send_can_frame: Callable):
        """
        Initialize ODrive device.
        """
        pass

    def make_arbitration_id(self, node_id: int, cmd_id: int) -> int:
        """
        Create the arbitration ID for a CAN message.
        """
        return (node_id << Arbitration.NODE_SIZE) | cmd_id

    def set_axis_state(self, state: AxisState) -> bool:
        """
        Set the state of the axis.
        """
        pass

    def set_controller_mode(self, mode: ControlMode, input_mode: InputMode) -> bool:
        """
        Set the controller mode of the axis.
        """
        pass

    def set_position(
        self, pos: float, velocity_ff: float = 0.0, torque_ff: float = 0.0
    ) -> bool:
        """
        Set the position of the axis.
        """
        pass

    def set_velocity(self, vel: float, torque_ff: float = 0.0) -> bool:
        """
        Set the velocity of the axis.
        """
        pass

    def set_torques(self, torque: float) -> bool:
        """
        Set the torque of the axis.
        """
        pass

    def request_encoder_estimates(self) -> bool:
        """
        Request the encoder estimates.
        """
        pass

    def clear_errors(self) -> bool:
        """
        Clear the errors on the ODrive.
        """
        pass

    def handler_encoder_estimates(self, data: bytes) -> None:
        """
        Handle the encoder estimates. Apply direction and turns to the position and velocity.

        Args:
            raw data: Data received from the ODrive.
        """
        pass

    def handle_encoder_estimates(self, data: bytes) -> None:
        """
        Process encoder estimates message
        """
        pass

    def handle_heartbeat(self, data: bytes) -> None:
        """
        Process heartbeat message
        """
        pass

    def handle_torque(self, data: bytes) -> None:
        """
        Process torque messages
        """
        pass

    def process_can_message(self, cmd: int, data: bytes) -> None:
        """
        Process CAN message
        """
        pass
