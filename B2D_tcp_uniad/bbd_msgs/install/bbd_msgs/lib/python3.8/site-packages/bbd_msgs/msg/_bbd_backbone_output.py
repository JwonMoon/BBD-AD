# generated from rosidl_generator_py/resource/_idl.py.em
# with input from bbd_msgs:msg/BBDBackboneOutput.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'cnn_feature'
# Member 'measurement_feature'
# Member 'traj_hidden_state'
import array  # noqa: E402, I100

# Member 'target_point'
# Member 'command'
# Member 'pred_wp'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BBDBackboneOutput(type):
    """Metaclass of message 'BBDBackboneOutput'."""

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
            module = import_type_support('bbd_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'bbd_msgs.msg.BBDBackboneOutput')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__bbd_backbone_output
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__bbd_backbone_output
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__bbd_backbone_output
            cls._TYPE_SUPPORT = module.type_support_msg__msg__bbd_backbone_output
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__bbd_backbone_output

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class BBDBackboneOutput(metaclass=Metaclass_BBDBackboneOutput):
    """Message class 'BBDBackboneOutput'."""

    __slots__ = [
        '_cnn_feature',
        '_measurement_feature',
        '_traj_hidden_state',
        '_speed',
        '_gt_velocity',
        '_target_point',
        '_command',
        '_pred_wp',
        '_step',
    ]

    _fields_and_field_types = {
        'cnn_feature': 'sequence<float>',
        'measurement_feature': 'sequence<float>',
        'traj_hidden_state': 'sequence<float>',
        'speed': 'float',
        'gt_velocity': 'float',
        'target_point': 'float[2]',
        'command': 'float[6]',
        'pred_wp': 'float[8]',
        'step': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 2),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 6),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 8),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.cnn_feature = array.array('f', kwargs.get('cnn_feature', []))
        self.measurement_feature = array.array('f', kwargs.get('measurement_feature', []))
        self.traj_hidden_state = array.array('f', kwargs.get('traj_hidden_state', []))
        self.speed = kwargs.get('speed', float())
        self.gt_velocity = kwargs.get('gt_velocity', float())
        if 'target_point' not in kwargs:
            self.target_point = numpy.zeros(2, dtype=numpy.float32)
        else:
            self.target_point = numpy.array(kwargs.get('target_point'), dtype=numpy.float32)
            assert self.target_point.shape == (2, )
        if 'command' not in kwargs:
            self.command = numpy.zeros(6, dtype=numpy.float32)
        else:
            self.command = numpy.array(kwargs.get('command'), dtype=numpy.float32)
            assert self.command.shape == (6, )
        if 'pred_wp' not in kwargs:
            self.pred_wp = numpy.zeros(8, dtype=numpy.float32)
        else:
            self.pred_wp = numpy.array(kwargs.get('pred_wp'), dtype=numpy.float32)
            assert self.pred_wp.shape == (8, )
        self.step = kwargs.get('step', int())

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
        if self.cnn_feature != other.cnn_feature:
            return False
        if self.measurement_feature != other.measurement_feature:
            return False
        if self.traj_hidden_state != other.traj_hidden_state:
            return False
        if self.speed != other.speed:
            return False
        if self.gt_velocity != other.gt_velocity:
            return False
        if all(self.target_point != other.target_point):
            return False
        if all(self.command != other.command):
            return False
        if all(self.pred_wp != other.pred_wp):
            return False
        if self.step != other.step:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def cnn_feature(self):
        """Message field 'cnn_feature'."""
        return self._cnn_feature

    @cnn_feature.setter
    def cnn_feature(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'cnn_feature' array.array() must have the type code of 'f'"
            self._cnn_feature = value
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
                 True), \
                "The 'cnn_feature' field must be a set or sequence and each value of type 'float'"
        self._cnn_feature = array.array('f', value)

    @property
    def measurement_feature(self):
        """Message field 'measurement_feature'."""
        return self._measurement_feature

    @measurement_feature.setter
    def measurement_feature(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'measurement_feature' array.array() must have the type code of 'f'"
            self._measurement_feature = value
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
                 True), \
                "The 'measurement_feature' field must be a set or sequence and each value of type 'float'"
        self._measurement_feature = array.array('f', value)

    @property
    def traj_hidden_state(self):
        """Message field 'traj_hidden_state'."""
        return self._traj_hidden_state

    @traj_hidden_state.setter
    def traj_hidden_state(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'traj_hidden_state' array.array() must have the type code of 'f'"
            self._traj_hidden_state = value
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
                 True), \
                "The 'traj_hidden_state' field must be a set or sequence and each value of type 'float'"
        self._traj_hidden_state = array.array('f', value)

    @property
    def speed(self):
        """Message field 'speed'."""
        return self._speed

    @speed.setter
    def speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed' field must be of type 'float'"
        self._speed = value

    @property
    def gt_velocity(self):
        """Message field 'gt_velocity'."""
        return self._gt_velocity

    @gt_velocity.setter
    def gt_velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'gt_velocity' field must be of type 'float'"
        self._gt_velocity = value

    @property
    def target_point(self):
        """Message field 'target_point'."""
        return self._target_point

    @target_point.setter
    def target_point(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'target_point' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 2, \
                "The 'target_point' numpy.ndarray() must have a size of 2"
            self._target_point = value
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
                 len(value) == 2 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'target_point' field must be a set or sequence with length 2 and each value of type 'float'"
        self._target_point = numpy.array(value, dtype=numpy.float32)

    @property
    def command(self):
        """Message field 'command'."""
        return self._command

    @command.setter
    def command(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'command' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 6, \
                "The 'command' numpy.ndarray() must have a size of 6"
            self._command = value
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
                 len(value) == 6 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'command' field must be a set or sequence with length 6 and each value of type 'float'"
        self._command = numpy.array(value, dtype=numpy.float32)

    @property
    def pred_wp(self):
        """Message field 'pred_wp'."""
        return self._pred_wp

    @pred_wp.setter
    def pred_wp(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float32, \
                "The 'pred_wp' numpy.ndarray() must have the dtype of 'numpy.float32'"
            assert value.size == 8, \
                "The 'pred_wp' numpy.ndarray() must have a size of 8"
            self._pred_wp = value
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
                 len(value) == 8 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'pred_wp' field must be a set or sequence with length 8 and each value of type 'float'"
        self._pred_wp = numpy.array(value, dtype=numpy.float32)

    @property
    def step(self):
        """Message field 'step'."""
        return self._step

    @step.setter
    def step(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'step' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'step' field must be an integer in [-2147483648, 2147483647]"
        self._step = value
