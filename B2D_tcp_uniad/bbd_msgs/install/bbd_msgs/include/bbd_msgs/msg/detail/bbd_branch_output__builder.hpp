// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from bbd_msgs:msg/BBDBranchOutput.idl
// generated code does not contain a copyright notice

#ifndef BBD_MSGS__MSG__DETAIL__BBD_BRANCH_OUTPUT__BUILDER_HPP_
#define BBD_MSGS__MSG__DETAIL__BBD_BRANCH_OUTPUT__BUILDER_HPP_

#include "bbd_msgs/msg/detail/bbd_branch_output__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace bbd_msgs
{

namespace msg
{

namespace builder
{

class Init_BBDBranchOutput_step
{
public:
  explicit Init_BBDBranchOutput_step(::bbd_msgs::msg::BBDBranchOutput & msg)
  : msg_(msg)
  {}
  ::bbd_msgs::msg::BBDBranchOutput step(::bbd_msgs::msg::BBDBranchOutput::_step_type arg)
  {
    msg_.step = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBranchOutput msg_;
};

class Init_BBDBranchOutput_manual_gear_shift
{
public:
  explicit Init_BBDBranchOutput_manual_gear_shift(::bbd_msgs::msg::BBDBranchOutput & msg)
  : msg_(msg)
  {}
  Init_BBDBranchOutput_step manual_gear_shift(::bbd_msgs::msg::BBDBranchOutput::_manual_gear_shift_type arg)
  {
    msg_.manual_gear_shift = std::move(arg);
    return Init_BBDBranchOutput_step(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBranchOutput msg_;
};

class Init_BBDBranchOutput_gear
{
public:
  explicit Init_BBDBranchOutput_gear(::bbd_msgs::msg::BBDBranchOutput & msg)
  : msg_(msg)
  {}
  Init_BBDBranchOutput_manual_gear_shift gear(::bbd_msgs::msg::BBDBranchOutput::_gear_type arg)
  {
    msg_.gear = std::move(arg);
    return Init_BBDBranchOutput_manual_gear_shift(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBranchOutput msg_;
};

class Init_BBDBranchOutput_reverse
{
public:
  explicit Init_BBDBranchOutput_reverse(::bbd_msgs::msg::BBDBranchOutput & msg)
  : msg_(msg)
  {}
  Init_BBDBranchOutput_gear reverse(::bbd_msgs::msg::BBDBranchOutput::_reverse_type arg)
  {
    msg_.reverse = std::move(arg);
    return Init_BBDBranchOutput_gear(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBranchOutput msg_;
};

class Init_BBDBranchOutput_hand_brake
{
public:
  explicit Init_BBDBranchOutput_hand_brake(::bbd_msgs::msg::BBDBranchOutput & msg)
  : msg_(msg)
  {}
  Init_BBDBranchOutput_reverse hand_brake(::bbd_msgs::msg::BBDBranchOutput::_hand_brake_type arg)
  {
    msg_.hand_brake = std::move(arg);
    return Init_BBDBranchOutput_reverse(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBranchOutput msg_;
};

class Init_BBDBranchOutput_brake
{
public:
  explicit Init_BBDBranchOutput_brake(::bbd_msgs::msg::BBDBranchOutput & msg)
  : msg_(msg)
  {}
  Init_BBDBranchOutput_hand_brake brake(::bbd_msgs::msg::BBDBranchOutput::_brake_type arg)
  {
    msg_.brake = std::move(arg);
    return Init_BBDBranchOutput_hand_brake(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBranchOutput msg_;
};

class Init_BBDBranchOutput_steer
{
public:
  explicit Init_BBDBranchOutput_steer(::bbd_msgs::msg::BBDBranchOutput & msg)
  : msg_(msg)
  {}
  Init_BBDBranchOutput_brake steer(::bbd_msgs::msg::BBDBranchOutput::_steer_type arg)
  {
    msg_.steer = std::move(arg);
    return Init_BBDBranchOutput_brake(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBranchOutput msg_;
};

class Init_BBDBranchOutput_throttle
{
public:
  explicit Init_BBDBranchOutput_throttle(::bbd_msgs::msg::BBDBranchOutput & msg)
  : msg_(msg)
  {}
  Init_BBDBranchOutput_steer throttle(::bbd_msgs::msg::BBDBranchOutput::_throttle_type arg)
  {
    msg_.throttle = std::move(arg);
    return Init_BBDBranchOutput_steer(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBranchOutput msg_;
};

class Init_BBDBranchOutput_header
{
public:
  Init_BBDBranchOutput_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BBDBranchOutput_throttle header(::bbd_msgs::msg::BBDBranchOutput::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BBDBranchOutput_throttle(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBranchOutput msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::bbd_msgs::msg::BBDBranchOutput>()
{
  return bbd_msgs::msg::builder::Init_BBDBranchOutput_header();
}

}  // namespace bbd_msgs

#endif  // BBD_MSGS__MSG__DETAIL__BBD_BRANCH_OUTPUT__BUILDER_HPP_
