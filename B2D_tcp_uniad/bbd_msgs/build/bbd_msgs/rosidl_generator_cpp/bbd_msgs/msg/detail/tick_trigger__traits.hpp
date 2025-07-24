// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from bbd_msgs:msg/TickTrigger.idl
// generated code does not contain a copyright notice

#ifndef BBD_MSGS__MSG__DETAIL__TICK_TRIGGER__TRAITS_HPP_
#define BBD_MSGS__MSG__DETAIL__TICK_TRIGGER__TRAITS_HPP_

#include "bbd_msgs/msg/detail/tick_trigger__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<bbd_msgs::msg::TickTrigger>()
{
  return "bbd_msgs::msg::TickTrigger";
}

template<>
inline const char * name<bbd_msgs::msg::TickTrigger>()
{
  return "bbd_msgs/msg/TickTrigger";
}

template<>
struct has_fixed_size<bbd_msgs::msg::TickTrigger>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<bbd_msgs::msg::TickTrigger>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<bbd_msgs::msg::TickTrigger>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BBD_MSGS__MSG__DETAIL__TICK_TRIGGER__TRAITS_HPP_
