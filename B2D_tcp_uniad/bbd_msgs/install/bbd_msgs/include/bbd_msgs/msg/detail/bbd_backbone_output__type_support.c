// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from bbd_msgs:msg/BBDBackboneOutput.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "bbd_msgs/msg/detail/bbd_backbone_output__rosidl_typesupport_introspection_c.h"
#include "bbd_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "bbd_msgs/msg/detail/bbd_backbone_output__functions.h"
#include "bbd_msgs/msg/detail/bbd_backbone_output__struct.h"


// Include directives for member types
// Member `cnn_feature`
// Member `measurement_feature`
// Member `traj_hidden_state`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void BBDBackboneOutput__rosidl_typesupport_introspection_c__BBDBackboneOutput_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  bbd_msgs__msg__BBDBackboneOutput__init(message_memory);
}

void BBDBackboneOutput__rosidl_typesupport_introspection_c__BBDBackboneOutput_fini_function(void * message_memory)
{
  bbd_msgs__msg__BBDBackboneOutput__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember BBDBackboneOutput__rosidl_typesupport_introspection_c__BBDBackboneOutput_message_member_array[9] = {
  {
    "cnn_feature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBackboneOutput, cnn_feature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "measurement_feature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBackboneOutput, measurement_feature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "traj_hidden_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBackboneOutput, traj_hidden_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBackboneOutput, speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gt_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBackboneOutput, gt_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_point",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBackboneOutput, target_point),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "command",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBackboneOutput, command),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pred_wp",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    8,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBackboneOutput, pred_wp),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "step",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBackboneOutput, step),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers BBDBackboneOutput__rosidl_typesupport_introspection_c__BBDBackboneOutput_message_members = {
  "bbd_msgs__msg",  // message namespace
  "BBDBackboneOutput",  // message name
  9,  // number of fields
  sizeof(bbd_msgs__msg__BBDBackboneOutput),
  BBDBackboneOutput__rosidl_typesupport_introspection_c__BBDBackboneOutput_message_member_array,  // message members
  BBDBackboneOutput__rosidl_typesupport_introspection_c__BBDBackboneOutput_init_function,  // function to initialize message memory (memory has to be allocated)
  BBDBackboneOutput__rosidl_typesupport_introspection_c__BBDBackboneOutput_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t BBDBackboneOutput__rosidl_typesupport_introspection_c__BBDBackboneOutput_message_type_support_handle = {
  0,
  &BBDBackboneOutput__rosidl_typesupport_introspection_c__BBDBackboneOutput_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_bbd_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, bbd_msgs, msg, BBDBackboneOutput)() {
  if (!BBDBackboneOutput__rosidl_typesupport_introspection_c__BBDBackboneOutput_message_type_support_handle.typesupport_identifier) {
    BBDBackboneOutput__rosidl_typesupport_introspection_c__BBDBackboneOutput_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &BBDBackboneOutput__rosidl_typesupport_introspection_c__BBDBackboneOutput_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
