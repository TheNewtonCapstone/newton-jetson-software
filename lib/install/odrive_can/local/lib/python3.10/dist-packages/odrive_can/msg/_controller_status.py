# generated from rosidl_generator_py/resource/_idl.py.em
# with input from odrive_can:msg/ControllerStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ControllerStatus(type):
    """Metaclass of message 'ControllerStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('odrive_can')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'odrive_can.msg.ControllerStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__controller_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__controller_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__controller_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__controller_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__controller_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ControllerStatus(metaclass=Metaclass_ControllerStatus):
    """Message class 'ControllerStatus'."""

    __slots__ = [
        '_pos_estimate',
        '_vel_estimate',
        '_torque_target',
        '_torque_estimate',
        '_iq_setpoint',
        '_iq_measured',
        '_active_errors',
        '_axis_state',
        '_procedure_result',
        '_trajectory_done_flag',
    ]

    _fields_and_field_types = {
        'pos_estimate': 'float',
        'vel_estimate': 'float',
        'torque_target': 'float',
        'torque_estimate': 'float',
        'iq_setpoint': 'float',
        'iq_measured': 'float',
        'active_errors': 'uint32',
        'axis_state': 'uint8',
        'procedure_result': 'uint8',
        'trajectory_done_flag': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.pos_estimate = kwargs.get('pos_estimate', float())
        self.vel_estimate = kwargs.get('vel_estimate', float())
        self.torque_target = kwargs.get('torque_target', float())
        self.torque_estimate = kwargs.get('torque_estimate', float())
        self.iq_setpoint = kwargs.get('iq_setpoint', float())
        self.iq_measured = kwargs.get('iq_measured', float())
        self.active_errors = kwargs.get('active_errors', int())
        self.axis_state = kwargs.get('axis_state', int())
        self.procedure_result = kwargs.get('procedure_result', int())
        self.trajectory_done_flag = kwargs.get('trajectory_done_flag', bool())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.pos_estimate != other.pos_estimate:
            return False
        if self.vel_estimate != other.vel_estimate:
            return False
        if self.torque_target != other.torque_target:
            return False
        if self.torque_estimate != other.torque_estimate:
            return False
        if self.iq_setpoint != other.iq_setpoint:
            return False
        if self.iq_measured != other.iq_measured:
            return False
        if self.active_errors != other.active_errors:
            return False
        if self.axis_state != other.axis_state:
            return False
        if self.procedure_result != other.procedure_result:
            return False
        if self.trajectory_done_flag != other.trajectory_done_flag:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pos_estimate(self):
        """Message field 'pos_estimate'."""
        return self._pos_estimate

    @pos_estimate.setter
    def pos_estimate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pos_estimate' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pos_estimate' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pos_estimate = value

    @builtins.property
    def vel_estimate(self):
        """Message field 'vel_estimate'."""
        return self._vel_estimate

    @vel_estimate.setter
    def vel_estimate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vel_estimate' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vel_estimate' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vel_estimate = value

    @builtins.property
    def torque_target(self):
        """Message field 'torque_target'."""
        return self._torque_target

    @torque_target.setter
    def torque_target(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'torque_target' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'torque_target' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._torque_target = value

    @builtins.property
    def torque_estimate(self):
        """Message field 'torque_estimate'."""
        return self._torque_estimate

    @torque_estimate.setter
    def torque_estimate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'torque_estimate' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'torque_estimate' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._torque_estimate = value

    @builtins.property
    def iq_setpoint(self):
        """Message field 'iq_setpoint'."""
        return self._iq_setpoint

    @iq_setpoint.setter
    def iq_setpoint(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'iq_setpoint' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'iq_setpoint' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._iq_setpoint = value

    @builtins.property
    def iq_measured(self):
        """Message field 'iq_measured'."""
        return self._iq_measured

    @iq_measured.setter
    def iq_measured(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'iq_measured' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'iq_measured' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._iq_measured = value

    @builtins.property
    def active_errors(self):
        """Message field 'active_errors'."""
        return self._active_errors

    @active_errors.setter
    def active_errors(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'active_errors' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'active_errors' field must be an unsigned integer in [0, 4294967295]"
        self._active_errors = value

    @builtins.property
    def axis_state(self):
        """Message field 'axis_state'."""
        return self._axis_state

    @axis_state.setter
    def axis_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'axis_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'axis_state' field must be an unsigned integer in [0, 255]"
        self._axis_state = value

    @builtins.property
    def procedure_result(self):
        """Message field 'procedure_result'."""
        return self._procedure_result

    @procedure_result.setter
    def procedure_result(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'procedure_result' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'procedure_result' field must be an unsigned integer in [0, 255]"
        self._procedure_result = value

    @builtins.property
    def trajectory_done_flag(self):
        """Message field 'trajectory_done_flag'."""
        return self._trajectory_done_flag

    @trajectory_done_flag.setter
    def trajectory_done_flag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'trajectory_done_flag' field must be of type 'bool'"
        self._trajectory_done_flag = value
