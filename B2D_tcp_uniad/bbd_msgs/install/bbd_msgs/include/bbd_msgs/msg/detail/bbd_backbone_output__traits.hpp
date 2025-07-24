// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from bbd_msgs:msg/BBDBackboneOutput.idl
// generated code does not contain a copyright notice

#ifndef BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__TRAITS_HPP_
#define BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__TRAITS_HPP_

#include "bbd_msgs/msg/detail/bbd_backbone_output__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<bbd_msgs::msg::BBDBackboneOutput>()
{
  return "bbd_msgs::msg::BBDBackboneOutput";
}

template<>
inline const char * name<bbd_msgs::msg::BBDBackboneOutput>()
{
  return "bbd_msgs/msg/BBDBackboneOutput";
}

template<>
struct has_fixed_size<bbd_msgs::msg::BBDBackboneOutput>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<bbd_msgs::msg::BBDBackboneOutput>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<bbd_msgs::msg::BBDBackboneOutput>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__TRAITS_HPP_
