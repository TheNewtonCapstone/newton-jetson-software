# generated from rosidl_generator_py/resource/_idl.py.em
# with input from odrive_can:msg/ODriveStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ODriveStatus(type):
    """Metaclass of message 'ODriveStatus'."""

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
                'odrive_can.msg.ODriveStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__o_drive_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__o_drive_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__o_drive_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__o_drive_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__o_drive_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ODriveStatus(metaclass=Metaclass_ODriveStatus):
    """Message class 'ODriveStatus'."""

    __slots__ = [
        '_bus_voltage',
        '_bus_current',
        '_fet_temperature',
        '_motor_temperature',
        '_active_errors',
        '_disarm_reason',
    ]

    _fields_and_field_types = {
        'bus_voltage': 'float',
        'bus_current': 'float',
        'fet_temperature': 'float',
        'motor_temperature': 'float',
        'active_errors': 'uint32',
        'disarm_reason': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.bus_voltage = kwargs.get('bus_voltage', float())
        self.bus_current = kwargs.get('bus_current', float())
        self.fet_temperature = kwargs.get('fet_temperature', float())
        self.motor_temperature = kwargs.get('motor_temperature', float())
        self.active_errors = kwargs.get('active_errors', int())
        self.disarm_reason = kwargs.get('disarm_reason', int())

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
        if self.bus_voltage != other.bus_voltage:
            return False
        if self.bus_current != other.bus_current:
            return False
        if self.fet_temperature != other.fet_temperature:
            return False
        if self.motor_temperature != other.motor_temperature:
            return False
        if self.active_errors != other.active_errors:
            return False
        if self.disarm_reason != other.disarm_reason:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def bus_voltage(self):
        """Message field 'bus_voltage'."""
        return self._bus_voltage

    @bus_voltage.setter
    def bus_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bus_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bus_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bus_voltage = value

    @builtins.property
    def bus_current(self):
        """Message field 'bus_current'."""
        return self._bus_current

    @bus_current.setter
    def bus_current(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'bus_current' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bus_current' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bus_current = value

    @builtins.property
    def fet_temperature(self):
        """Message field 'fet_temperature'."""
        return self._fet_temperature

    @fet_temperature.setter
    def fet_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'fet_temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'fet_temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._fet_temperature = value

    @builtins.property
    def motor_temperature(self):
        """Message field 'motor_temperature'."""
        return self._motor_temperature

    @motor_temperature.setter
    def motor_temperature(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'motor_temperature' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'motor_temperature' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._motor_temperature = value

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
    def disarm_reason(self):
        """Message field 'disarm_reason'."""
        return self._disarm_reason

    @disarm_reason.setter
    def disarm_reason(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'disarm_reason' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'disarm_reason' field must be an unsigned integer in [0, 4294967295]"
        self._disarm_reason = value
