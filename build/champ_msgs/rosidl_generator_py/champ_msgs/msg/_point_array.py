# generated from rosidl_generator_py/resource/_idl.py.em
# with input from champ_msgs:msg/PointArray.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PointArray(type):
    """Metaclass of message 'PointArray'."""

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
            module = import_type_support('champ_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'champ_msgs.msg.PointArray')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__point_array
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__point_array
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__point_array
            cls._TYPE_SUPPORT = module.type_support_msg__msg__point_array
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__point_array

            from champ_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PointArray(metaclass=Metaclass_PointArray):
    """Message class 'PointArray'."""

    __slots__ = [
        '_lf',
        '_rf',
        '_lh',
        '_rh',
    ]

    _fields_and_field_types = {
        'lf': 'champ_msgs/Point',
        'rf': 'champ_msgs/Point',
        'lh': 'champ_msgs/Point',
        'rh': 'champ_msgs/Point',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['champ_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['champ_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['champ_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['champ_msgs', 'msg'], 'Point'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from champ_msgs.msg import Point
        self.lf = kwargs.get('lf', Point())
        from champ_msgs.msg import Point
        self.rf = kwargs.get('rf', Point())
        from champ_msgs.msg import Point
        self.lh = kwargs.get('lh', Point())
        from champ_msgs.msg import Point
        self.rh = kwargs.get('rh', Point())

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
        if self.lf != other.lf:
            return False
        if self.rf != other.rf:
            return False
        if self.lh != other.lh:
            return False
        if self.rh != other.rh:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def lf(self):
        """Message field 'lf'."""
        return self._lf

    @lf.setter
    def lf(self, value):
        if __debug__:
            from champ_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'lf' field must be a sub message of type 'Point'"
        self._lf = value

    @builtins.property
    def rf(self):
        """Message field 'rf'."""
        return self._rf

    @rf.setter
    def rf(self, value):
        if __debug__:
            from champ_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'rf' field must be a sub message of type 'Point'"
        self._rf = value

    @builtins.property
    def lh(self):
        """Message field 'lh'."""
        return self._lh

    @lh.setter
    def lh(self, value):
        if __debug__:
            from champ_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'lh' field must be a sub message of type 'Point'"
        self._lh = value

    @builtins.property
    def rh(self):
        """Message field 'rh'."""
        return self._rh

    @rh.setter
    def rh(self, value):
        if __debug__:
            from champ_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'rh' field must be a sub message of type 'Point'"
        self._rh = value
