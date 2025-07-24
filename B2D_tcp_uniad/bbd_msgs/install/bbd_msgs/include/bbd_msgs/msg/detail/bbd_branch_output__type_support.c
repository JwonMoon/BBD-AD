// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from bbd_msgs:msg/BBDBranchOutput.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "bbd_msgs/msg/detail/bbd_branch_output__rosidl_typesupport_introspection_c.h"
#include "bbd_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "bbd_msgs/msg/detail/bbd_branch_output__functions.h"
#include "bbd_msgs/msg/detail/bbd_branch_output__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void BBDBranchOutput__rosidl_typesupport_introspection_c__BBDBranchOutput_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  bbd_msgs__msg__BBDBranchOutput__init(message_memory);
}

void BBDBranchOutput__rosidl_typesupport_introspection_c__BBDBranchOutput_fini_function(void * message_memory)
{
  bbd_msgs__msg__BBDBranchOutput__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember BBDBranchOutput__rosidl_typesupport_introspection_c__BBDBranchOutput_message_member_array[9] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBranchOutput, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "throttle",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBranchOutput, throttle),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "steer",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBranchOutput, steer),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "brake",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBranchOutput, brake),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "hand_brake",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBranchOutput, hand_brake),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reverse",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBranchOutput, reverse),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "gear",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBranchOutput, gear),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "manual_gear_shift",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__BBDBranchOutput, manual_gear_shift),  // bytes offset in struct
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
    offsetof(bbd_msgs__msg__BBDBranchOutput, step),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers BBDBranchOutput__rosidl_typesupport_introspection_c__BBDBranchOutput_message_members = {
  "bbd_msgs__msg",  // message namespace
  "BBDBranchOutput",  // message name
  9,  // number of fields
  sizeof(bbd_msgs__msg__BBDBranchOutput),
  BBDBranchOutput__rosidl_typesupport_introspection_c__BBDBranchOutput_message_member_array,  // message members
  BBDBranchOutput__rosidl_typesupport_introspection_c__BBDBranchOutput_init_function,  // function to initialize message memory (memory has to be allocated)
  BBDBranchOutput__rosidl_typesupport_introspection_c__BBDBranchOutput_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t BBDBranchOutput__rosidl_typesupport_introspection_c__BBDBranchOutput_message_type_support_handle = {
  0,
  &BBDBranchOutput__rosidl_typesupport_introspection_c__BBDBranchOutput_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_bbd_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, bbd_msgs, msg, BBDBranchOutput)() {
  BBDBranchOutput__rosidl_typesupport_introspection_c__BBDBranchOutput_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!BBDBranchOutput__rosidl_typesupport_introspection_c__BBDBranchOutput_message_type_support_handle.typesupport_identifier) {
    BBDBranchOutput__rosidl_typesupport_introspection_c__BBDBranchOutput_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &BBDBranchOutput__rosidl_typesupport_introspection_c__BBDBranchOutput_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
