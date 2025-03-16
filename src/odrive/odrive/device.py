import struct
from enum import IntEnum, unique
from typing import Callable, Dict, Optional, List, Tuple, Any
import time
from .can_interface import Arbitration

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
    ENCODER_HALL_PHASE_receive_thread = 12
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

    SET_CONTROLLER_MODE = 0x0B
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
    ENTER_DFU_MODE = 0x1F


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
    This class represents a single ODrive device on the CAN bus.
    """

    def __init__(self, node_id: int, send_can_frame: Callable[[int, bytes], bool]):
        """
        Initialize ODrive device.
        
        Args:
            node_id: CAN node ID of the ODrive device
            send_can_frame: Function to send CAN frames (takes arbitration_id and data)
        """
        self.node_id = node_id
        self.send_can_frame = send_can_frame
        
        # State tracking
        self.position = 0.0
        self.velocity = 0.0
        self.torque_target = 0.0
        self.torque_estimate = 0.0
        self.axis_error = 0
        self.axis_state = AxisState.UNDEFINED
        self.procedure_result = 0
        self.trajectory_done = False
        
        # Message handlers
        self.message_handlers = {
            OdriveCANCommands.GET_ENCODER_ESTIMATES: self.handle_encoder_estimates,
            OdriveCANCommands.GET_HEARTBEAT: self.handle_heartbeat,
            OdriveCANCommands.GET_TORQUE: self.handle_torque,
        }
        
        # Debug info
        self.last_send_time = 0
        self.last_receive_time = 0

    def make_arbitration_id(self, node_id: int, cmd_id: int) -> int:
        """
        Create the arbitration ID for a CAN message.
        
        Args:
            node_id: Node ID of the target ODrive
            cmd_id: Command ID of the message
            
        Returns:
            int: Arbitration ID
        """
        return (node_id << Arbitration.NODE_ID_SIZE) | cmd_id

    def set_axis_state(self, state: AxisState) -> bool:
        """
        Set the state of the axis.
        
        Args:
            state: Target axis state
            
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_AXIS_STATE
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack('<I', int(state))
        success = self.send_can_frame(arb_id, data)
        
        if success:
            self.last_send_time = time.time()
        
        return success

    def set_controller_mode(self, control_mode: ControlMode, input_mode: InputMode) -> bool:
        """
        Set the controller mode of the axis.
        
        Args:
            control_mode: Control mode to set
            input_mode: Input mode to set
            
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_CONTROLLER_MODE
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack('<II', int(control_mode), int(input_mode))
        success = self.send_can_frame(arb_id, data)
        
        if success:
            self.last_send_time = time.time()
        
        return success

    def set_position(self, pos: float, velocity_ff: float = 0.0, torque_ff: float = 0.0) -> bool:
        """
        Set the position of the axis.
        
        Args:
            pos: Target position in turns
            velocity_ff: Velocity feed-forward in turns/s
            torque_ff: Torque feed-forward in Nm
            
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_INPUT_POS
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        
        # Convert to appropriate format
        vel_ff_int = int(velocity_ff * 1000)  # Scale by 1000 per ODrive protocol
        torque_ff_int = int(torque_ff * 1000)  # Scale by 1000 per ODrive protocol
        
        # Clamp to int16 range
        vel_ff_int = max(min(vel_ff_int, 32767), -32768)
        torque_ff_int = max(min(torque_ff_int, 32767), -32768)
        
        data = struct.pack('<fhh', pos, vel_ff_int, torque_ff_int)
        success = self.send_can_frame(arb_id, data)
        
        if success:
            self.last_send_time = time.time()
        
        return success

    def set_velocity(self, vel: float, torque_ff: float = 0.0) -> bool:
        """
        Set the velocity of the axis.
        
        Args:
            vel: Target velocity in turns/s
            torque_ff: Torque feed-forward in Nm
            
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_INPUT_VEL
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack('<ff', vel, torque_ff)
        success = self.send_can_frame(arb_id, data)
        
        if success:
            self.last_send_time = time.time()
        
        return success

    def set_torque(self, torque: float) -> bool:
        """
        Set the torque of the axis.
        
        Args:
            torque: Target torque in Nm
            
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_INPUT_TORQUE
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack('<f', torque)
        success = self.send_can_frame(arb_id, data)
        
        if success:
            self.last_send_time = time.time()
        
        return success

    def request_encoder_estimates(self) -> bool:
        """
        Request the encoder estimates.
        
        Returns:
            bool: True if request was sent successfully
        """
        cmd_id = OdriveCANCommands.GET_ENCODER_ESTIMATES
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        # Send an RTR frame (Remote Transmission Request)
        # Using empty data with RTR bit would be better, but our interface doesn't expose RTR
        # so we'll send a regular frame with empty data
        success = self.send_can_frame(arb_id, b'')
        
        if success:
            self.last_send_time = time.time()
        
        return success

    def request_heartbeat(self) -> bool:
        """
        Request a heartbeat message.
        
        Returns:
            bool: True if request was sent successfully
        """
        cmd_id = OdriveCANCommands.GET_HEARTBEAT
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        success = self.send_can_frame(arb_id, b'')
        
        if success:
            self.last_send_time = time.time()
        
        return success

    def clear_errors(self, identify: bool = False) -> bool:
        """
        Clear the errors on the ODrive.
        
        Args:
            identify: If True, blink the LED to identify the ODrive
            
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.CLEAR_ERRORS
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack('<B', 1 if identify else 0)
        success = self.send_can_frame(arb_id, data)
        
        if success:
            self.last_send_time = time.time()
        
        return success

    def handle_encoder_estimates(self, data: bytes) -> None:
        """
        Process encoder estimates message
        
        Args:
            data: Data received from the ODrive
        """
        if len(data) >= 8:
            pos, vel = struct.unpack('<ff', data[:8])
            self.position = pos
            self.velocity = vel
            self.last_receive_time = time.time()

    def handle_heartbeat(self, data: bytes) -> None:
        """
        Process heartbeat message
        
        Args:
            data: Data received from the ODrive
        """
        if len(data) >= 7:
            error, state, procedure_result, trajectory_done = struct.unpack('<IBBB', data[:7])
            self.axis_error = error
            self.axis_state = AxisState(state) if state in [e.value for e in AxisState] else AxisState.UNDEFINED
            self.procedure_result = procedure_result
            self.trajectory_done = (trajectory_done == 1)
            self.last_receive_time = time.time()

    def handle_torque(self, data: bytes) -> None:
        """
        Process torque message
        
        Args:
            data: Data received from the ODrive
        """
        if len(data) >= 8:
            torque_target, torque_estimate = struct.unpack('<ff', data[:8])
            self.torque_target = torque_target
            self.torque_estimate = torque_estimate
            self.last_receive_time = time.time()

    def process_can_message(self, cmd_id: int, data: bytes) -> None:
        """
            Process CAN message from the ODrive
        
        Args:
            cmd_id: Command ID of the message
            data: Data received from the ODrive
        """
        handler = self.message_handlers.get(cmd_id)
        print(f"[red]Received message from {self.node_id} with cmd_id {cmd_id}[/red]")
        
        if handler:
            handler(data)
            
    def set_limits(self, velocity_limit: float, current_limit: float) -> bool:
        """
        Set velocity and current limits
        
        Args:
            velocity_limit: Maximum velocity in turns/s
            current_limit: Maximum current in Amps
            
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_LIMITS
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack('<ff', velocity_limit, current_limit)
        success = self.send_can_frame(arb_id, data)
        
        if success:
            self.last_send_time = time.time()
        
        return success
    
    def reboot(self, save_config: bool = False) -> bool:
        """
        Reboot the ODrive
        
        Args:
            save_config: If True, save configuration before rebooting
            
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.REBOOT
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        action = 1 if save_config else 0  # 0: reboot, 1: save_configuration()
        data = struct.pack('<B', action)
        success = self.send_can_frame(arb_id, data)
        
        if success:
            self.last_send_time = time.time()
        
        return success
    def set_absolute_position(self, position: float) -> bool:
        """
        Sets the encoder position to a specified value
        
        Args:
            position: Desired position in turns
            
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_ABSOLUTE_POSITION
        arb_id = self.make_arbitration_id(self.node_id, cmd_id) 
        data = struct.pack('<f', position)
        success = self.send_can_frame(arb_id, data)
        
        if success:
            self.last_send_time = time.time()
        
        return success
    
    def estop(self) -> bool:
        """
        Emergency stop (disarm the ODrive)
        
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.ESTOP
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        success = self.send_can_frame(arb_id, b'')
        
        if success:
            self.last_send_time = time.time()
        
        return success
    
    def set_pos_gain(self, pos_gain: float) -> bool:
        """
        Set the position gain (P gain)
        
        Args:
            pos_gain: Position gain in (turns/s)/turns
            
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_POS_GAINS
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack('<f', pos_gain)
        success = self.send_can_frame(arb_id, data)
        
        if success:
            self.last_send_time = time.time()
        
        return success
    
    def set_vel_gains(self, vel_gain: float, vel_integrator_gain: float) -> bool:
        """
        Set the velocity gains (P and I gains)
        
        Args:
            vel_gain: Velocity gain in Nm/(turns/s)
            vel_integrator_gain: Velocity integrator gain in Nm/turns
            
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_VEL_GAINS
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack('<ff', vel_gain, vel_integrator_gain)
        success = self.send_can_frame(arb_id, data)
        
        if success:
            self.last_send_time = time.time()
        
        return success
    
    def set_traj_vel_limit(self, vel_limit: float) -> bool:
        """
        Set the trajectory velocity limit
        
        Args:
            vel_limit: Velocity limit in turns/s
            
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_TRAJ_VEL_LIMIT
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack('<f', vel_limit)
        success = self.send_can_frame(arb_id, data)
        
        if success:
            self.last_send_time = time.time()
        
        return success
    
    def set_traj_accel_limits(self, accel_limit: float, decel_limit: float) -> bool:
        """
        Set the trajectory acceleration and deceleration limits
        
        Args:
            accel_limit: Acceleration limit in turns/s²
            decel_limit: Deceleration limit in turns/s²
            
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_TRAJ_ACCEL_LIMIT
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack('<ff', accel_limit, decel_limit)
        success = self.send_can_frame(arb_id, data)
        
        if success:
            self.last_send_time = time.time()
        
        return success
    
    def set_traj_inertia(self, inertia: float) -> bool:
        """
        Set the trajectory inertia
        
        Args:
            inertia: Inertia in Nm/(turns/s²)
            
        Returns:
            bool: True if message was sent successfully
        """
        cmd_id = OdriveCANCommands.SET_TRAJ_INERTIA
        arb_id = self.make_arbitration_id(self.node_id, cmd_id)
        data = struct.pack('<f', inertia)
        success = self.send_can_frame(arb_id, data)
        
        if success:
            self.last_send_time = time.time()
        
        return success
    
    def set_address_msg(self, sn: int, node_id: int) -> bool:
        # to be implemented, setting addressees using odrive's serial number
        pass 
    def get_address_msg(self, sn: int) -> bool:
        # to be implemented, getting addressees using odrive's serial number
        pass