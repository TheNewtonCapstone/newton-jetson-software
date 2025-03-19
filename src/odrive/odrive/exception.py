import logging
from enum import IntEnum
from error import ODriveErrorCode, ODriveProcedureResult


    
    
class CanException(Exception):
    def __init__(self, *args):
        super().__init__(*args) 
    pass
class NotConnectedException(CanException):
    pass



class ODriveException(Exception):
    def __init__(self, node_id: int, str: str, *args):
        self.node_id = node_id,
        self.message = f"Device {node_id}: {str}"
        super().__init__(self.message, *args) 
        
    
    def _log_exceptions(self):
        logging.error("An exception: {}".format(self))

class NoHearBeatException(ODriveException):
    def __init__(
        self, 
        node_id: int,
        error_code: ODriveErrorCode,
        procedure_result: ODriveProcedureResult,
        *args
        ):
        super().__init__(*args)

class TimeoutException(ODriveException):
    pass

class PositionCommandException(ODriveException):
    pass