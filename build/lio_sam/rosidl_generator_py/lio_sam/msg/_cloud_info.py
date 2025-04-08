# generated from rosidl_generator_py/resource/_idl.py.em
# with input from lio_sam:msg/CloudInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'start_ring_index'
# Member 'end_ring_index'
# Member 'point_col_ind'
# Member 'point_range'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CloudInfo(type):
    """Metaclass of message 'CloudInfo'."""

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
            module = import_type_support('lio_sam')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'lio_sam.msg.CloudInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__cloud_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__cloud_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__cloud_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__cloud_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__cloud_info

            from sensor_msgs.msg import PointCloud2
            if PointCloud2.__class__._TYPE_SUPPORT is None:
                PointCloud2.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CloudInfo(metaclass=Metaclass_CloudInfo):
    """Message class 'CloudInfo'."""

    __slots__ = [
        '_header',
        '_start_ring_index',
        '_end_ring_index',
        '_point_col_ind',
        '_point_range',
        '_imu_available',
        '_odom_available',
        '_imu_roll_init',
        '_imu_pitch_init',
        '_imu_yaw_init',
        '_initial_guess_x',
        '_initial_guess_y',
        '_initial_guess_z',
        '_initial_guess_roll',
        '_initial_guess_pitch',
        '_initial_guess_yaw',
        '_cloud_deskewed',
        '_cloud_corner',
        '_cloud_surface',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'start_ring_index': 'sequence<int32>',
        'end_ring_index': 'sequence<int32>',
        'point_col_ind': 'sequence<int32>',
        'point_range': 'sequence<float>',
        'imu_available': 'int64',
        'odom_available': 'int64',
        'imu_roll_init': 'float',
        'imu_pitch_init': 'float',
        'imu_yaw_init': 'float',
        'initial_guess_x': 'float',
        'initial_guess_y': 'float',
        'initial_guess_z': 'float',
        'initial_guess_roll': 'float',
        'initial_guess_pitch': 'float',
        'initial_guess_yaw': 'float',
        'cloud_deskewed': 'sensor_msgs/PointCloud2',
        'cloud_corner': 'sensor_msgs/PointCloud2',
        'cloud_surface': 'sensor_msgs/PointCloud2',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('int64'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'PointCloud2'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'PointCloud2'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'PointCloud2'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.start_ring_index = array.array('i', kwargs.get('start_ring_index', []))
        self.end_ring_index = array.array('i', kwargs.get('end_ring_index', []))
        self.point_col_ind = array.array('i', kwargs.get('point_col_ind', []))
        self.point_range = array.array('f', kwargs.get('point_range', []))
        self.imu_available = kwargs.get('imu_available', int())
        self.odom_available = kwargs.get('odom_available', int())
        self.imu_roll_init = kwargs.get('imu_roll_init', float())
        self.imu_pitch_init = kwargs.get('imu_pitch_init', float())
        self.imu_yaw_init = kwargs.get('imu_yaw_init', float())
        self.initial_guess_x = kwargs.get('initial_guess_x', float())
        self.initial_guess_y = kwargs.get('initial_guess_y', float())
        self.initial_guess_z = kwargs.get('initial_guess_z', float())
        self.initial_guess_roll = kwargs.get('initial_guess_roll', float())
        self.initial_guess_pitch = kwargs.get('initial_guess_pitch', float())
        self.initial_guess_yaw = kwargs.get('initial_guess_yaw', float())
        from sensor_msgs.msg import PointCloud2
        self.cloud_deskewed = kwargs.get('cloud_deskewed', PointCloud2())
        from sensor_msgs.msg import PointCloud2
        self.cloud_corner = kwargs.get('cloud_corner', PointCloud2())
        from sensor_msgs.msg import PointCloud2
        self.cloud_surface = kwargs.get('cloud_surface', PointCloud2())

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
        if self.header != other.header:
            return False
        if self.start_ring_index != other.start_ring_index:
            return False
        if self.end_ring_index != other.end_ring_index:
            return False
        if self.point_col_ind != other.point_col_ind:
            return False
        if self.point_range != other.point_range:
            return False
        if self.imu_available != other.imu_available:
            return False
        if self.odom_available != other.odom_available:
            return False
        if self.imu_roll_init != other.imu_roll_init:
            return False
        if self.imu_pitch_init != other.imu_pitch_init:
            return False
        if self.imu_yaw_init != other.imu_yaw_init:
            return False
        if self.initial_guess_x != other.initial_guess_x:
            return False
        if self.initial_guess_y != other.initial_guess_y:
            return False
        if self.initial_guess_z != other.initial_guess_z:
            return False
        if self.initial_guess_roll != other.initial_guess_roll:
            return False
        if self.initial_guess_pitch != other.initial_guess_pitch:
            return False
        if self.initial_guess_yaw != other.initial_guess_yaw:
            return False
        if self.cloud_deskewed != other.cloud_deskewed:
            return False
        if self.cloud_corner != other.cloud_corner:
            return False
        if self.cloud_surface != other.cloud_surface:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def start_ring_index(self):
        """Message field 'start_ring_index'."""
        return self._start_ring_index

    @start_ring_index.setter
    def start_ring_index(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'start_ring_index' array.array() must have the type code of 'i'"
            self._start_ring_index = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'start_ring_index' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._start_ring_index = array.array('i', value)

    @builtins.property
    def end_ring_index(self):
        """Message field 'end_ring_index'."""
        return self._end_ring_index

    @end_ring_index.setter
    def end_ring_index(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'end_ring_index' array.array() must have the type code of 'i'"
            self._end_ring_index = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'end_ring_index' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._end_ring_index = array.array('i', value)

    @builtins.property
    def point_col_ind(self):
        """Message field 'point_col_ind'."""
        return self._point_col_ind

    @point_col_ind.setter
    def point_col_ind(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'point_col_ind' array.array() must have the type code of 'i'"
            self._point_col_ind = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'point_col_ind' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._point_col_ind = array.array('i', value)

    @builtins.property
    def point_range(self):
        """Message field 'point_range'."""
        return self._point_range

    @point_range.setter
    def point_range(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'point_range' array.array() must have the type code of 'f'"
            self._point_range = value
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
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'point_range' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._point_range = array.array('f', value)

    @builtins.property
    def imu_available(self):
        """Message field 'imu_available'."""
        return self._imu_available

    @imu_available.setter
    def imu_available(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'imu_available' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'imu_available' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._imu_available = value

    @builtins.property
    def odom_available(self):
        """Message field 'odom_available'."""
        return self._odom_available

    @odom_available.setter
    def odom_available(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'odom_available' field must be of type 'int'"
            assert value >= -9223372036854775808 and value < 9223372036854775808, \
                "The 'odom_available' field must be an integer in [-9223372036854775808, 9223372036854775807]"
        self._odom_available = value

    @builtins.property
    def imu_roll_init(self):
        """Message field 'imu_roll_init'."""
        return self._imu_roll_init

    @imu_roll_init.setter
    def imu_roll_init(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_roll_init' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'imu_roll_init' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._imu_roll_init = value

    @builtins.property
    def imu_pitch_init(self):
        """Message field 'imu_pitch_init'."""
        return self._imu_pitch_init

    @imu_pitch_init.setter
    def imu_pitch_init(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_pitch_init' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'imu_pitch_init' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._imu_pitch_init = value

    @builtins.property
    def imu_yaw_init(self):
        """Message field 'imu_yaw_init'."""
        return self._imu_yaw_init

    @imu_yaw_init.setter
    def imu_yaw_init(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'imu_yaw_init' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'imu_yaw_init' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._imu_yaw_init = value

    @builtins.property
    def initial_guess_x(self):
        """Message field 'initial_guess_x'."""
        return self._initial_guess_x

    @initial_guess_x.setter
    def initial_guess_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'initial_guess_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'initial_guess_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._initial_guess_x = value

    @builtins.property
    def initial_guess_y(self):
        """Message field 'initial_guess_y'."""
        return self._initial_guess_y

    @initial_guess_y.setter
    def initial_guess_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'initial_guess_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'initial_guess_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._initial_guess_y = value

    @builtins.property
    def initial_guess_z(self):
        """Message field 'initial_guess_z'."""
        return self._initial_guess_z

    @initial_guess_z.setter
    def initial_guess_z(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'initial_guess_z' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'initial_guess_z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._initial_guess_z = value

    @builtins.property
    def initial_guess_roll(self):
        """Message field 'initial_guess_roll'."""
        return self._initial_guess_roll

    @initial_guess_roll.setter
    def initial_guess_roll(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'initial_guess_roll' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'initial_guess_roll' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._initial_guess_roll = value

    @builtins.property
    def initial_guess_pitch(self):
        """Message field 'initial_guess_pitch'."""
        return self._initial_guess_pitch

    @initial_guess_pitch.setter
    def initial_guess_pitch(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'initial_guess_pitch' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'initial_guess_pitch' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._initial_guess_pitch = value

    @builtins.property
    def initial_guess_yaw(self):
        """Message field 'initial_guess_yaw'."""
        return self._initial_guess_yaw

    @initial_guess_yaw.setter
    def initial_guess_yaw(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'initial_guess_yaw' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'initial_guess_yaw' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._initial_guess_yaw = value

    @builtins.property
    def cloud_deskewed(self):
        """Message field 'cloud_deskewed'."""
        return self._cloud_deskewed

    @cloud_deskewed.setter
    def cloud_deskewed(self, value):
        if __debug__:
            from sensor_msgs.msg import PointCloud2
            assert \
                isinstance(value, PointCloud2), \
                "The 'cloud_deskewed' field must be a sub message of type 'PointCloud2'"
        self._cloud_deskewed = value

    @builtins.property
    def cloud_corner(self):
        """Message field 'cloud_corner'."""
        return self._cloud_corner

    @cloud_corner.setter
    def cloud_corner(self, value):
        if __debug__:
            from sensor_msgs.msg import PointCloud2
            assert \
                isinstance(value, PointCloud2), \
                "The 'cloud_corner' field must be a sub message of type 'PointCloud2'"
        self._cloud_corner = value

    @builtins.property
    def cloud_surface(self):
        """Message field 'cloud_surface'."""
        return self._cloud_surface

    @cloud_surface.setter
    def cloud_surface(self, value):
        if __debug__:
            from sensor_msgs.msg import PointCloud2
            assert \
                isinstance(value, PointCloud2), \
                "The 'cloud_surface' field must be a sub message of type 'PointCloud2'"
        self._cloud_surface = value
