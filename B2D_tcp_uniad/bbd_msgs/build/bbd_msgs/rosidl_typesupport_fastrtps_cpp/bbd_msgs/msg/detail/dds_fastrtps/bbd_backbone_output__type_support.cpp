// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from bbd_msgs:msg/BBDBackboneOutput.idl
// generated code does not contain a copyright notice
#include "bbd_msgs/msg/detail/bbd_backbone_output__rosidl_typesupport_fastrtps_cpp.hpp"
#include "bbd_msgs/msg/detail/bbd_backbone_output__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace bbd_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bbd_msgs
cdr_serialize(
  const bbd_msgs::msg::BBDBackboneOutput & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: cnn_feature
  {
    cdr << ros_message.cnn_feature;
  }
  // Member: measurement_feature
  {
    cdr << ros_message.measurement_feature;
  }
  // Member: traj_hidden_state
  {
    cdr << ros_message.traj_hidden_state;
  }
  // Member: speed
  cdr << ros_message.speed;
  // Member: gt_velocity
  cdr << ros_message.gt_velocity;
  // Member: target_point
  {
    cdr << ros_message.target_point;
  }
  // Member: command
  {
    cdr << ros_message.command;
  }
  // Member: pred_wp
  {
    cdr << ros_message.pred_wp;
  }
  // Member: step
  cdr << ros_message.step;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bbd_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  bbd_msgs::msg::BBDBackboneOutput & ros_message)
{
  // Member: cnn_feature
  {
    cdr >> ros_message.cnn_feature;
  }

  // Member: measurement_feature
  {
    cdr >> ros_message.measurement_feature;
  }

  // Member: traj_hidden_state
  {
    cdr >> ros_message.traj_hidden_state;
  }

  // Member: speed
  cdr >> ros_message.speed;

  // Member: gt_velocity
  cdr >> ros_message.gt_velocity;

  // Member: target_point
  {
    cdr >> ros_message.target_point;
  }

  // Member: command
  {
    cdr >> ros_message.command;
  }

  // Member: pred_wp
  {
    cdr >> ros_message.pred_wp;
  }

  // Member: step
  cdr >> ros_message.step;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bbd_msgs
get_serialized_size(
  const bbd_msgs::msg::BBDBackboneOutput & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: cnn_feature
  {
    size_t array_size = ros_message.cnn_feature.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.cnn_feature[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: measurement_feature
  {
    size_t array_size = ros_message.measurement_feature.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.measurement_feature[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: traj_hidden_state
  {
    size_t array_size = ros_message.traj_hidden_state.size();

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.traj_hidden_state[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speed
  {
    size_t item_size = sizeof(ros_message.speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gt_velocity
  {
    size_t item_size = sizeof(ros_message.gt_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: target_point
  {
    size_t array_size = 2;
    size_t item_size = sizeof(ros_message.target_point[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: command
  {
    size_t array_size = 6;
    size_t item_size = sizeof(ros_message.command[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pred_wp
  {
    size_t array_size = 8;
    size_t item_size = sizeof(ros_message.pred_wp[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: step
  {
    size_t item_size = sizeof(ros_message.step);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bbd_msgs
max_serialized_size_BBDBackboneOutput(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: cnn_feature
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: measurement_feature
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: traj_hidden_state
  {
    size_t array_size = 0;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: speed
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: gt_velocity
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: target_point
  {
    size_t array_size = 2;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: command
  {
    size_t array_size = 6;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pred_wp
  {
    size_t array_size = 8;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: step
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _BBDBackboneOutput__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const bbd_msgs::msg::BBDBackboneOutput *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _BBDBackboneOutput__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<bbd_msgs::msg::BBDBackboneOutput *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _BBDBackboneOutput__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const bbd_msgs::msg::BBDBackboneOutput *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _BBDBackboneOutput__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_BBDBackboneOutput(full_bounded, 0);
}

static message_type_support_callbacks_t _BBDBackboneOutput__callbacks = {
  "bbd_msgs::msg",
  "BBDBackboneOutput",
  _BBDBackboneOutput__cdr_serialize,
  _BBDBackboneOutput__cdr_deserialize,
  _BBDBackboneOutput__get_serialized_size,
  _BBDBackboneOutput__max_serialized_size
};

static rosidl_message_type_support_t _BBDBackboneOutput__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_BBDBackboneOutput__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace bbd_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_bbd_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<bbd_msgs::msg::BBDBackboneOutput>()
{
  return &bbd_msgs::msg::typesupport_fastrtps_cpp::_BBDBackboneOutput__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, bbd_msgs, msg, BBDBackboneOutput)() {
  return &bbd_msgs::msg::typesupport_fastrtps_cpp::_BBDBackboneOutput__handle;
}

#ifdef __cplusplus
}
#endif
