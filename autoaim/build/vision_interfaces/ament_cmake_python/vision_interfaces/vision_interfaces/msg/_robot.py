# generated from rosidl_generator_py/resource/_idl.py.em
# with input from vision_interfaces:msg/Robot.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Robot(type):
    """Metaclass of message 'Robot'."""

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
            module = import_type_support('vision_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'vision_interfaces.msg.Robot')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robot
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robot
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robot
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robot
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robot

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Robot(metaclass=Metaclass_Robot):
    """Message class 'Robot'."""

    __slots__ = [
        '_mode',
        '_foe_color',
        '_self_yaw',
        '_self_pitch',
        '_muzzle_speed',
    ]

    _fields_and_field_types = {
        'mode': 'uint8',
        'foe_color': 'uint8',
        'self_yaw': 'float',
        'self_pitch': 'float',
        'muzzle_speed': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mode = kwargs.get('mode', int())
        self.foe_color = kwargs.get('foe_color', int())
        self.self_yaw = kwargs.get('self_yaw', float())
        self.self_pitch = kwargs.get('self_pitch', float())
        self.muzzle_speed = kwargs.get('muzzle_speed', float())

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
        if self.mode != other.mode:
            return False
        if self.foe_color != other.foe_color:
            return False
        if self.self_yaw != other.self_yaw:
            return False
        if self.self_pitch != other.self_pitch:
            return False
        if self.muzzle_speed != other.muzzle_speed:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value

    @builtins.property
    def foe_color(self):
        """Message field 'foe_color'."""
        return self._foe_color

    @foe_color.setter
    def foe_color(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'foe_color' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'foe_color' field must be an unsigned integer in [0, 255]"
        self._foe_color = value

    @builtins.property
    def self_yaw(self):
        """Message field 'self_yaw'."""
        return self._self_yaw

    @self_yaw.setter
    def self_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'self_yaw' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'self_yaw' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._self_yaw = value

    @builtins.property
    def self_pitch(self):
        """Message field 'self_pitch'."""
        return self._self_pitch

    @self_pitch.setter
    def self_pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'self_pitch' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'self_pitch' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._self_pitch = value

    @builtins.property
    def muzzle_speed(self):
        """Message field 'muzzle_speed'."""
        return self._muzzle_speed

    @muzzle_speed.setter
    def muzzle_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'muzzle_speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'muzzle_speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._muzzle_speed = value
