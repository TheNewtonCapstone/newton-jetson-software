from enum import IntEnum

class ODriveErrorCode(IntEnum):
    # see https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Error for more details
    
    INITILIZATION_ERROR = 1
    SYSTEM_LEVEL = 2 
    TIMING_ERROR = 4
    MISSING_ESTIMATE = 8
    BAD_CONFIG = 16
    DRV_FAULT = 32
    MISSING_INPUT = 64
    DC_BUS_OVER_VOLTAGE = 256
    DC_BUS_UNDER_VOLTAGE = 512
    DC_BUS_OVER_CURRENT = 1024  # either 
    DC_BUS_OVER_REGEN_CURRENT = 2048
    CURRENT_LIMITATION = 4096
    MOTOR_OVER_TEMP = 8192
    INVERTER_OVER_TEMP = 16384
    VELOCITY_LIMIT_VIOLATION = 32768
    POSITION_LIMIT_VIOLATION = 65536
    WATCHDOG_TIMER_EXPIRED = 131072
    ESTOP_REQUESTED = 33554432
    SPINOUT_DETECTED = 67108864
    BRAKE_RESISTOR_DISARMED = 134217728
    THERMISTOR_DISCONNECTED = 1073741824 
    

    
class ODriveProcedureResult:    
    # https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.ProcedureResult
    SUCCESS = 0
    BUSY = 1
    CANCELLED = 2
    DISARMED = 3
    NO_RESPONSE = 4
    POLE_PAIR_CPR_MISMATCH = 5
    PHASE_RESISTANCE_OUT_OF_RANGE = 6
    PHASE_INDUCTANCE_OUT_OF_RANGE = 7
    UNBALANCED_PHASES = 8
    INVALID_MOTOR_TYPE = 9
    ILLEGAL_HALL_STATE = 10
    TIMEOUT = 11
    HOMING_WITHOUT_ENDSTOP = 12
    NOT_CALIBRATED = 14 
    NOT_CONVERGING = 15 # 
    """  
    The calibration did not converge.

    The measurements made during a calibration task did not reach sufficient statistical significance.

    For instance during AxisState.ENCODER_OFFSET_CALIBRATION the encoder did not move sufficiently to determine the direction between encoder and motor.

    Try changing the calibration parameters."""
    
    
    