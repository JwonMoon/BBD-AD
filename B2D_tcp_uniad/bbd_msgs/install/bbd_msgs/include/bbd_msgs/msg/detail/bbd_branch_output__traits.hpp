// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from bbd_msgs:msg/BBDBranchOutput.idl
// generated code does not contain a copyright notice

#ifndef BBD_MSGS__MSG__DETAIL__BBD_BRANCH_OUTPUT__TRAITS_HPP_
#define BBD_MSGS__MSG__DETAIL__BBD_BRANCH_OUTPUT__TRAITS_HPP_

#include "bbd_msgs/msg/detail/bbd_branch_output__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<bbd_msgs::msg::BBDBranchOutput>()
{
  return "bbd_msgs::msg::BBDBranchOutput";
}

template<>
inline const char * name<bbd_msgs::msg::BBDBranchOutput>()
{
  return "bbd_msgs/msg/BBDBranchOutput";
}

template<>
struct has_fixed_size<bbd_msgs::msg::BBDBranchOutput>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<bbd_msgs::msg::BBDBranchOutput>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<bbd_msgs::msg::BBDBranchOutput>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BBD_MSGS__MSG__DETAIL__BBD_BRANCH_OUTPUT__TRAITS_HPP_
