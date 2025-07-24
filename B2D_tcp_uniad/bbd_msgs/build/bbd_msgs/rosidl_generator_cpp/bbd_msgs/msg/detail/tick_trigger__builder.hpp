// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from bbd_msgs:msg/TickTrigger.idl
// generated code does not contain a copyright notice

#ifndef BBD_MSGS__MSG__DETAIL__TICK_TRIGGER__BUILDER_HPP_
#define BBD_MSGS__MSG__DETAIL__TICK_TRIGGER__BUILDER_HPP_

#include "bbd_msgs/msg/detail/tick_trigger__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace bbd_msgs
{

namespace msg
{

namespace builder
{

class Init_TickTrigger_trigger
{
public:
  explicit Init_TickTrigger_trigger(::bbd_msgs::msg::TickTrigger & msg)
  : msg_(msg)
  {}
  ::bbd_msgs::msg::TickTrigger trigger(::bbd_msgs::msg::TickTrigger::_trigger_type arg)
  {
    msg_.trigger = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bbd_msgs::msg::TickTrigger msg_;
};

class Init_TickTrigger_step
{
public:
  Init_TickTrigger_step()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TickTrigger_trigger step(::bbd_msgs::msg::TickTrigger::_step_type arg)
  {
    msg_.step = std::move(arg);
    return Init_TickTrigger_trigger(msg_);
  }

private:
  ::bbd_msgs::msg::TickTrigger msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::bbd_msgs::msg::TickTrigger>()
{
  return bbd_msgs::msg::builder::Init_TickTrigger_step();
}

}  // namespace bbd_msgs

#endif  // BBD_MSGS__MSG__DETAIL__TICK_TRIGGER__BUILDER_HPP_
