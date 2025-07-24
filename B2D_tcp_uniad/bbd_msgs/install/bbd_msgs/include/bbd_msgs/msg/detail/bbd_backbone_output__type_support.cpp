// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from bbd_msgs:msg/BBDBackboneOutput.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "bbd_msgs/msg/detail/bbd_backbone_output__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace bbd_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void BBDBackboneOutput_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) bbd_msgs::msg::BBDBackboneOutput(_init);
}

void BBDBackboneOutput_fini_function(void * message_memory)
{
  auto typed_message = static_cast<bbd_msgs::msg::BBDBackboneOutput *>(message_memory);
  typed_message->~BBDBackboneOutput();
}

size_t size_function__BBDBackboneOutput__cnn_feature(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BBDBackboneOutput__cnn_feature(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__BBDBackboneOutput__cnn_feature(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__BBDBackboneOutput__cnn_feature(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BBDBackboneOutput__measurement_feature(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BBDBackboneOutput__measurement_feature(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__BBDBackboneOutput__measurement_feature(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__BBDBackboneOutput__measurement_feature(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BBDBackboneOutput__traj_hidden_state(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__BBDBackboneOutput__traj_hidden_state(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__BBDBackboneOutput__traj_hidden_state(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__BBDBackboneOutput__traj_hidden_state(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__BBDBackboneOutput__target_point(const void * untyped_member)
{
  (void)untyped_member;
  return 2;
}

const void * get_const_function__BBDBackboneOutput__target_point(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 2> *>(untyped_member);
  return &member[index];
}

void * get_function__BBDBackboneOutput__target_point(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 2> *>(untyped_member);
  return &member[index];
}

size_t size_function__BBDBackboneOutput__command(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__BBDBackboneOutput__command(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__BBDBackboneOutput__command(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 6> *>(untyped_member);
  return &member[index];
}

size_t size_function__BBDBackboneOutput__pred_wp(const void * untyped_member)
{
  (void)untyped_member;
  return 8;
}

const void * get_const_function__BBDBackboneOutput__pred_wp(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 8> *>(untyped_member);
  return &member[index];
}

void * get_function__BBDBackboneOutput__pred_wp(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 8> *>(untyped_member);
  return &member[index];
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember BBDBackboneOutput_message_member_array[9] = {
  {
    "cnn_feature",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs::msg::BBDBackboneOutput, cnn_feature),  // bytes offset in struct
    nullptr,  // default value
    size_function__BBDBackboneOutput__cnn_feature,  // size() function pointer
    get_const_function__BBDBackboneOutput__cnn_feature,  // get_const(index) function pointer
    get_function__BBDBackboneOutput__cnn_feature,  // get(index) function pointer
    resize_function__BBDBackboneOutput__cnn_feature  // resize(index) function pointer
  },
  {
    "measurement_feature",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs::msg::BBDBackboneOutput, measurement_feature),  // bytes offset in struct
    nullptr,  // default value
    size_function__BBDBackboneOutput__measurement_feature,  // size() function pointer
    get_const_function__BBDBackboneOutput__measurement_feature,  // get_const(index) function pointer
    get_function__BBDBackboneOutput__measurement_feature,  // get(index) function pointer
    resize_function__BBDBackboneOutput__measurement_feature  // resize(index) function pointer
  },
  {
    "traj_hidden_state",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs::msg::BBDBackboneOutput, traj_hidden_state),  // bytes offset in struct
    nullptr,  // default value
    size_function__BBDBackboneOutput__traj_hidden_state,  // size() function pointer
    get_const_function__BBDBackboneOutput__traj_hidden_state,  // get_const(index) function pointer
    get_function__BBDBackboneOutput__traj_hidden_state,  // get(index) function pointer
    resize_function__BBDBackboneOutput__traj_hidden_state  // resize(index) function pointer
  },
  {
    "speed",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs::msg::BBDBackboneOutput, speed),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "gt_velocity",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs::msg::BBDBackboneOutput, gt_velocity),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "target_point",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    2,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs::msg::BBDBackboneOutput, target_point),  // bytes offset in struct
    nullptr,  // default value
    size_function__BBDBackboneOutput__target_point,  // size() function pointer
    get_const_function__BBDBackboneOutput__target_point,  // get_const(index) function pointer
    get_function__BBDBackboneOutput__target_point,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "command",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs::msg::BBDBackboneOutput, command),  // bytes offset in struct
    nullptr,  // default value
    size_function__BBDBackboneOutput__command,  // size() function pointer
    get_const_function__BBDBackboneOutput__command,  // get_const(index) function pointer
    get_function__BBDBackboneOutput__command,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pred_wp",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    8,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs::msg::BBDBackboneOutput, pred_wp),  // bytes offset in struct
    nullptr,  // default value
    size_function__BBDBackboneOutput__pred_wp,  // size() function pointer
    get_const_function__BBDBackboneOutput__pred_wp,  // get_const(index) function pointer
    get_function__BBDBackboneOutput__pred_wp,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "step",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(bbd_msgs::msg::BBDBackboneOutput, step),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers BBDBackboneOutput_message_members = {
  "bbd_msgs::msg",  // message namespace
  "BBDBackboneOutput",  // message name
  9,  // number of fields
  sizeof(bbd_msgs::msg::BBDBackboneOutput),
  BBDBackboneOutput_message_member_array,  // message members
  BBDBackboneOutput_init_function,  // function to initialize message memory (memory has to be allocated)
  BBDBackboneOutput_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t BBDBackboneOutput_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &BBDBackboneOutput_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace bbd_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<bbd_msgs::msg::BBDBackboneOutput>()
{
  return &::bbd_msgs::msg::rosidl_typesupport_introspection_cpp::BBDBackboneOutput_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, bbd_msgs, msg, BBDBackboneOutput)() {
  return &::bbd_msgs::msg::rosidl_typesupport_introspection_cpp::BBDBackboneOutput_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
