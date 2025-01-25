# generated from rosidl_generator_py/resource/_idl.py.em
# with input from odrive_can:srv/AxisState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AxisState_Request(type):
    """Metaclass of message 'AxisState_Request'."""

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
                'odrive_can.srv.AxisState_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__axis_state__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__axis_state__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__axis_state__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__axis_state__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__axis_state__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AxisState_Request(metaclass=Metaclass_AxisState_Request):
    """Message class 'AxisState_Request'."""

    __slots__ = [
        '_axis_requested_state',
    ]

    _fields_and_field_types = {
        'axis_requested_state': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.axis_requested_state = kwargs.get('axis_requested_state', int())

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
        if self.axis_requested_state != other.axis_requested_state:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def axis_requested_state(self):
        """Message field 'axis_requested_state'."""
        return self._axis_requested_state

    @axis_requested_state.setter
    def axis_requested_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'axis_requested_state' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'axis_requested_state' field must be an unsigned integer in [0, 4294967295]"
        self._axis_requested_state = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_AxisState_Response(type):
    """Metaclass of message 'AxisState_Response'."""

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
                'odrive_can.srv.AxisState_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__axis_state__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__axis_state__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__axis_state__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__axis_state__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__axis_state__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AxisState_Response(metaclass=Metaclass_AxisState_Response):
    """Message class 'AxisState_Response'."""

    __slots__ = [
        '_active_errors',
        '_axis_state',
        '_procedure_result',
    ]

    _fields_and_field_types = {
        'active_errors': 'uint32',
        'axis_state': 'uint8',
        'procedure_result': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.active_errors = kwargs.get('active_errors', int())
        self.axis_state = kwargs.get('axis_state', int())
        self.procedure_result = kwargs.get('procedure_result', int())

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
        if self.active_errors != other.active_errors:
            return False
        if self.axis_state != other.axis_state:
            return False
        if self.procedure_result != other.procedure_result:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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


class Metaclass_AxisState(type):
    """Metaclass of service 'AxisState'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('odrive_can')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'odrive_can.srv.AxisState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__axis_state

            from odrive_can.srv import _axis_state
            if _axis_state.Metaclass_AxisState_Request._TYPE_SUPPORT is None:
                _axis_state.Metaclass_AxisState_Request.__import_type_support__()
            if _axis_state.Metaclass_AxisState_Response._TYPE_SUPPORT is None:
                _axis_state.Metaclass_AxisState_Response.__import_type_support__()


class AxisState(metaclass=Metaclass_AxisState):
    from odrive_can.srv._axis_state import AxisState_Request as Request
    from odrive_can.srv._axis_state import AxisState_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
