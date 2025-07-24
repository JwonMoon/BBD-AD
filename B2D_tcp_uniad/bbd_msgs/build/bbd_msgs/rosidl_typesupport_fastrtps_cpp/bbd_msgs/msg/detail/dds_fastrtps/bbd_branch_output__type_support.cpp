// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from bbd_msgs:msg/BBDBranchOutput.idl
// generated code does not contain a copyright notice
#include "bbd_msgs/msg/detail/bbd_branch_output__rosidl_typesupport_fastrtps_cpp.hpp"
#include "bbd_msgs/msg/detail/bbd_branch_output__struct.hpp"

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
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace bbd_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bbd_msgs
cdr_serialize(
  const bbd_msgs::msg::BBDBranchOutput & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: throttle
  cdr << ros_message.throttle;
  // Member: steer
  cdr << ros_message.steer;
  // Member: brake
  cdr << ros_message.brake;
  // Member: hand_brake
  cdr << (ros_message.hand_brake ? true : false);
  // Member: reverse
  cdr << (ros_message.reverse ? true : false);
  // Member: gear
  cdr << ros_message.gear;
  // Member: manual_gear_shift
  cdr << (ros_message.manual_gear_shift ? true : false);
  // Member: step
  cdr << ros_message.step;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bbd_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  bbd_msgs::msg::BBDBranchOutput & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: throttle
  cdr >> ros_message.throttle;

  // Member: steer
  cdr >> ros_message.steer;

  // Member: brake
  cdr >> ros_message.brake;

  // Member: hand_brake
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.hand_brake = tmp ? true : false;
  }

  // Member: reverse
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.reverse = tmp ? true : false;
  }

  // Member: gear
  cdr >> ros_message.gear;

  // Member: manual_gear_shift
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.manual_gear_shift = tmp ? true : false;
  }

  // Member: step
  cdr >> ros_message.step;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_bbd_msgs
get_serialized_size(
  const bbd_msgs::msg::BBDBranchOutput & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: throttle
  {
    size_t item_size = sizeof(ros_message.throttle);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: steer
  {
    size_t item_size = sizeof(ros_message.steer);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: brake
  {
    size_t item_size = sizeof(ros_message.brake);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: hand_brake
  {
    size_t item_size = sizeof(ros_message.hand_brake);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reverse
  {
    size_t item_size = sizeof(ros_message.reverse);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: gear
  {
    size_t item_size = sizeof(ros_message.gear);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: manual_gear_shift
  {
    size_t item_size = sizeof(ros_message.manual_gear_shift);
    current_alignment += item_size +
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
max_serialized_size_BBDBranchOutput(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: header
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        full_bounded, current_alignment);
    }
  }

  // Member: throttle
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: steer
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: brake
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: hand_brake
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: reverse
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: gear
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: manual_gear_shift
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: step
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _BBDBranchOutput__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const bbd_msgs::msg::BBDBranchOutput *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _BBDBranchOutput__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<bbd_msgs::msg::BBDBranchOutput *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _BBDBranchOutput__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const bbd_msgs::msg::BBDBranchOutput *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _BBDBranchOutput__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_BBDBranchOutput(full_bounded, 0);
}

static message_type_support_callbacks_t _BBDBranchOutput__callbacks = {
  "bbd_msgs::msg",
  "BBDBranchOutput",
  _BBDBranchOutput__cdr_serialize,
  _BBDBranchOutput__cdr_deserialize,
  _BBDBranchOutput__get_serialized_size,
  _BBDBranchOutput__max_serialized_size
};

static rosidl_message_type_support_t _BBDBranchOutput__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_BBDBranchOutput__callbacks,
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
get_message_type_support_handle<bbd_msgs::msg::BBDBranchOutput>()
{
  return &bbd_msgs::msg::typesupport_fastrtps_cpp::_BBDBranchOutput__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, bbd_msgs, msg, BBDBranchOutput)() {
  return &bbd_msgs::msg::typesupport_fastrtps_cpp::_BBDBranchOutput__handle;
}

#ifdef __cplusplus
}
#endif
