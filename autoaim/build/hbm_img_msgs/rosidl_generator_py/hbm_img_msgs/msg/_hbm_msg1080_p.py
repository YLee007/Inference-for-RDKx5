# generated from rosidl_generator_py/resource/_idl.py.em
# with input from hbm_img_msgs:msg/HbmMsg1080P.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

# Member 'encoding'
# Member 'data'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_HbmMsg1080P(type):
    """Metaclass of message 'HbmMsg1080P'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MAX_SIZE': 6220800,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('hbm_img_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'hbm_img_msgs.msg.HbmMsg1080P')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__hbm_msg1080_p
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__hbm_msg1080_p
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__hbm_msg1080_p
            cls._TYPE_SUPPORT = module.type_support_msg__msg__hbm_msg1080_p
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__hbm_msg1080_p

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MAX_SIZE': cls.__constants['MAX_SIZE'],
        }

    @property
    def MAX_SIZE(self):
        """Message constant 'MAX_SIZE'."""
        return Metaclass_HbmMsg1080P.__constants['MAX_SIZE']


class HbmMsg1080P(metaclass=Metaclass_HbmMsg1080P):
    """
    Message class 'HbmMsg1080P'.

    Constants:
      MAX_SIZE
    """

    __slots__ = [
        '_index',
        '_time_stamp',
        '_height',
        '_width',
        '_data_size',
        '_step',
        '_encoding',
        '_data',
    ]

    _fields_and_field_types = {
        'index': 'int32',
        'time_stamp': 'builtin_interfaces/Time',
        'height': 'uint32',
        'width': 'uint32',
        'data_size': 'uint32',
        'step': 'uint32',
        'encoding': 'uint8[12]',
        'data': 'uint8[6220800]',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 12),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 6220800),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.index = kwargs.get('index', int())
        from builtin_interfaces.msg import Time
        self.time_stamp = kwargs.get('time_stamp', Time())
        self.height = kwargs.get('height', int())
        self.width = kwargs.get('width', int())
        self.data_size = kwargs.get('data_size', int())
        self.step = kwargs.get('step', int())
        if 'encoding' not in kwargs:
            self.encoding = numpy.zeros(12, dtype=numpy.uint8)
        else:
            self.encoding = numpy.array(kwargs.get('encoding'), dtype=numpy.uint8)
            assert self.encoding.shape == (12, )
        if 'data' not in kwargs:
            self.data = numpy.zeros(6220800, dtype=numpy.uint8)
        else:
            self.data = numpy.array(kwargs.get('data'), dtype=numpy.uint8)
            assert self.data.shape == (6220800, )

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
        if self.index != other.index:
            return False
        if self.time_stamp != other.time_stamp:
            return False
        if self.height != other.height:
            return False
        if self.width != other.width:
            return False
        if self.data_size != other.data_size:
            return False
        if self.step != other.step:
            return False
        if all(self.encoding != other.encoding):
            return False
        if all(self.data != other.data):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def index(self):
        """Message field 'index'."""
        return self._index

    @index.setter
    def index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'index' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'index' field must be an integer in [-2147483648, 2147483647]"
        self._index = value

    @builtins.property
    def time_stamp(self):
        """Message field 'time_stamp'."""
        return self._time_stamp

    @time_stamp.setter
    def time_stamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'time_stamp' field must be a sub message of type 'Time'"
        self._time_stamp = value

    @builtins.property
    def height(self):
        """Message field 'height'."""
        return self._height

    @height.setter
    def height(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'height' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'height' field must be an unsigned integer in [0, 4294967295]"
        self._height = value

    @builtins.property
    def width(self):
        """Message field 'width'."""
        return self._width

    @width.setter
    def width(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'width' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'width' field must be an unsigned integer in [0, 4294967295]"
        self._width = value

    @builtins.property
    def data_size(self):
        """Message field 'data_size'."""
        return self._data_size

    @data_size.setter
    def data_size(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'data_size' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'data_size' field must be an unsigned integer in [0, 4294967295]"
        self._data_size = value

    @builtins.property
    def step(self):
        """Message field 'step'."""
        return self._step

    @step.setter
    def step(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'step' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'step' field must be an unsigned integer in [0, 4294967295]"
        self._step = value

    @builtins.property
    def encoding(self):
        """Message field 'encoding'."""
        return self._encoding

    @encoding.setter
    def encoding(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'encoding' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 12, \
                "The 'encoding' numpy.ndarray() must have a size of 12"
            self._encoding = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 12 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'encoding' field must be a set or sequence with length 12 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._encoding = numpy.array(value, dtype=numpy.uint8)

    @builtins.property
    def data(self):
        """Message field 'data'."""
        return self._data

    @data.setter
    def data(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'data' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 6220800, \
                "The 'data' numpy.ndarray() must have a size of 6220800"
            self._data = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 6220800 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'data' field must be a set or sequence with length 6220800 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._data = numpy.array(value, dtype=numpy.uint8)
