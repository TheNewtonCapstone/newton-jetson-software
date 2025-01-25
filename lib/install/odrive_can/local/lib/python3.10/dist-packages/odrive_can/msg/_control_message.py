# generated from rosidl_generator_py/resource/_idl.py.em
# with input from odrive_can:msg/ControlMessage.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ControlMessage(type):
    """Metaclass of message 'ControlMessage'."""

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
                'odrive_can.msg.ControlMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__control_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__control_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__control_message
            cls._TYPE_SUPPORT = module.type_support_msg__msg__control_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__control_message

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ControlMessage(metaclass=Metaclass_ControlMessage):
    """Message class 'ControlMessage'."""

    __slots__ = [
        '_control_mode',
        '_input_mode',
        '_input_pos',
        '_input_vel',
        '_input_torque',
    ]

    _fields_and_field_types = {
        'control_mode': 'uint32',
        'input_mode': 'uint32',
        'input_pos': 'float',
        'input_vel': 'float',
        'input_torque': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.control_mode = kwargs.get('control_mode', int())
        self.input_mode = kwargs.get('input_mode', int())
        self.input_pos = kwargs.get('input_pos', float())
        self.input_vel = kwargs.get('input_vel', float())
        self.input_torque = kwargs.get('input_torque', float())

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
        if self.control_mode != other.control_mode:
            return False
        if self.input_mode != other.input_mode:
            return False
        if self.input_pos != other.input_pos:
            return False
        if self.input_vel != other.input_vel:
            return False
        if self.input_torque != other.input_torque:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def control_mode(self):
        """Message field 'control_mode'."""
        return self._control_mode

    @control_mode.setter
    def control_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'control_mode' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'control_mode' field must be an unsigned integer in [0, 4294967295]"
        self._control_mode = value

    @builtins.property
    def input_mode(self):
        """Message field 'input_mode'."""
        return self._input_mode

    @input_mode.setter
    def input_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'input_mode' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'input_mode' field must be an unsigned integer in [0, 4294967295]"
        self._input_mode = value

    @builtins.property
    def input_pos(self):
        """Message field 'input_pos'."""
        return self._input_pos

    @input_pos.setter
    def input_pos(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'input_pos' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'input_pos' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._input_pos = value

    @builtins.property
    def input_vel(self):
        """Message field 'input_vel'."""
        return self._input_vel

    @input_vel.setter
    def input_vel(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'input_vel' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'input_vel' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._input_vel = value

    @builtins.property
    def input_torque(self):
        """Message field 'input_torque'."""
        return self._input_torque

    @input_torque.setter
    def input_torque(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'input_torque' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'input_torque' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._input_torque = value
