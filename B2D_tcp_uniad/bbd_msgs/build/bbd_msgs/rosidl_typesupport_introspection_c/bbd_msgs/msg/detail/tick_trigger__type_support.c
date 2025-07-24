// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from bbd_msgs:msg/TickTrigger.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "bbd_msgs/msg/detail/tick_trigger__rosidl_typesupport_introspection_c.h"
#include "bbd_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "bbd_msgs/msg/detail/tick_trigger__functions.h"
#include "bbd_msgs/msg/detail/tick_trigger__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void TickTrigger__rosidl_typesupport_introspection_c__TickTrigger_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  bbd_msgs__msg__TickTrigger__init(message_memory);
}

void TickTrigger__rosidl_typesupport_introspection_c__TickTrigger_fini_function(void * message_memory)
{
  bbd_msgs__msg__TickTrigger__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember TickTrigger__rosidl_typesupport_introspection_c__TickTrigger_message_member_array[2] = {
  {
    "step",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__TickTrigger, step),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "trigger",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs__msg__TickTrigger, trigger),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers TickTrigger__rosidl_typesupport_introspection_c__TickTrigger_message_members = {
  "bbd_msgs__msg",  // message namespace
  "TickTrigger",  // message name
  2,  // number of fields
  sizeof(bbd_msgs__msg__TickTrigger),
  TickTrigger__rosidl_typesupport_introspection_c__TickTrigger_message_member_array,  // message members
  TickTrigger__rosidl_typesupport_introspection_c__TickTrigger_init_function,  // function to initialize message memory (memory has to be allocated)
  TickTrigger__rosidl_typesupport_introspection_c__TickTrigger_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t TickTrigger__rosidl_typesupport_introspection_c__TickTrigger_message_type_support_handle = {
  0,
  &TickTrigger__rosidl_typesupport_introspection_c__TickTrigger_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_bbd_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, bbd_msgs, msg, TickTrigger)() {
  if (!TickTrigger__rosidl_typesupport_introspection_c__TickTrigger_message_type_support_handle.typesupport_identifier) {
    TickTrigger__rosidl_typesupport_introspection_c__TickTrigger_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &TickTrigger__rosidl_typesupport_introspection_c__TickTrigger_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
