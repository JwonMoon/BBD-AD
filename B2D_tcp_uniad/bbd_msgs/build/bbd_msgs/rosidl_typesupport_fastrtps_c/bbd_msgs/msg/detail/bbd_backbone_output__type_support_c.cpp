// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from bbd_msgs:msg/BBDBackboneOutput.idl
// generated code does not contain a copyright notice
#include "bbd_msgs/msg/detail/bbd_backbone_output__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "bbd_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "bbd_msgs/msg/detail/bbd_backbone_output__struct.h"
#include "bbd_msgs/msg/detail/bbd_backbone_output__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/primitives_sequence.h"  // cnn_feature, measurement_feature, traj_hidden_state
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // cnn_feature, measurement_feature, traj_hidden_state

// forward declare type support functions


using _BBDBackboneOutput__ros_msg_type = bbd_msgs__msg__BBDBackboneOutput;

static bool _BBDBackboneOutput__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _BBDBackboneOutput__ros_msg_type * ros_message = static_cast<const _BBDBackboneOutput__ros_msg_type *>(untyped_ros_message);
  // Field name: cnn_feature
  {
    size_t size = ros_message->cnn_feature.size;
    auto array_ptr = ros_message->cnn_feature.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: measurement_feature
  {
    size_t size = ros_message->measurement_feature.size;
    auto array_ptr = ros_message->measurement_feature.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: traj_hidden_state
  {
    size_t size = ros_message->traj_hidden_state.size;
    auto array_ptr = ros_message->traj_hidden_state.data;
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: speed
  {
    cdr << ros_message->speed;
  }

  // Field name: gt_velocity
  {
    cdr << ros_message->gt_velocity;
  }

  // Field name: target_point
  {
    size_t size = 2;
    auto array_ptr = ros_message->target_point;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: command
  {
    size_t size = 6;
    auto array_ptr = ros_message->command;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: pred_wp
  {
    size_t size = 8;
    auto array_ptr = ros_message->pred_wp;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: step
  {
    cdr << ros_message->step;
  }

  return true;
}

static bool _BBDBackboneOutput__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _BBDBackboneOutput__ros_msg_type * ros_message = static_cast<_BBDBackboneOutput__ros_msg_type *>(untyped_ros_message);
  // Field name: cnn_feature
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->cnn_feature.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->cnn_feature);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->cnn_feature, size)) {
      return "failed to create array for field 'cnn_feature'";
    }
    auto array_ptr = ros_message->cnn_feature.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: measurement_feature
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->measurement_feature.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->measurement_feature);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->measurement_feature, size)) {
      return "failed to create array for field 'measurement_feature'";
    }
    auto array_ptr = ros_message->measurement_feature.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: traj_hidden_state
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->traj_hidden_state.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->traj_hidden_state);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->traj_hidden_state, size)) {
      return "failed to create array for field 'traj_hidden_state'";
    }
    auto array_ptr = ros_message->traj_hidden_state.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: speed
  {
    cdr >> ros_message->speed;
  }

  // Field name: gt_velocity
  {
    cdr >> ros_message->gt_velocity;
  }

  // Field name: target_point
  {
    size_t size = 2;
    auto array_ptr = ros_message->target_point;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: command
  {
    size_t size = 6;
    auto array_ptr = ros_message->command;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: pred_wp
  {
    size_t size = 8;
    auto array_ptr = ros_message->pred_wp;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: step
  {
    cdr >> ros_message->step;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bbd_msgs
size_t get_serialized_size_bbd_msgs__msg__BBDBackboneOutput(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _BBDBackboneOutput__ros_msg_type * ros_message = static_cast<const _BBDBackboneOutput__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name cnn_feature
  {
    size_t array_size = ros_message->cnn_feature.size;
    auto array_ptr = ros_message->cnn_feature.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name measurement_feature
  {
    size_t array_size = ros_message->measurement_feature.size;
    auto array_ptr = ros_message->measurement_feature.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name traj_hidden_state
  {
    size_t array_size = ros_message->traj_hidden_state.size;
    auto array_ptr = ros_message->traj_hidden_state.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speed
  {
    size_t item_size = sizeof(ros_message->speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name gt_velocity
  {
    size_t item_size = sizeof(ros_message->gt_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name target_point
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->target_point;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name command
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->command;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pred_wp
  {
    size_t array_size = 8;
    auto array_ptr = ros_message->pred_wp;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name step
  {
    size_t item_size = sizeof(ros_message->step);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _BBDBackboneOutput__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_bbd_msgs__msg__BBDBackboneOutput(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_bbd_msgs
size_t max_serialized_size_bbd_msgs__msg__BBDBackboneOutput(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: cnn_feature
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: measurement_feature
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: traj_hidden_state
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: gt_velocity
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: target_point
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: command
  {
    size_t array_size = 6;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: pred_wp
  {
    size_t array_size = 8;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: step
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _BBDBackboneOutput__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_bbd_msgs__msg__BBDBackboneOutput(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_BBDBackboneOutput = {
  "bbd_msgs::msg",
  "BBDBackboneOutput",
  _BBDBackboneOutput__cdr_serialize,
  _BBDBackboneOutput__cdr_deserialize,
  _BBDBackboneOutput__get_serialized_size,
  _BBDBackboneOutput__max_serialized_size
};

static rosidl_message_type_support_t _BBDBackboneOutput__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_BBDBackboneOutput,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, bbd_msgs, msg, BBDBackboneOutput)() {
  return &_BBDBackboneOutput__type_support;
}

#if defined(__cplusplus)
}
#endif
