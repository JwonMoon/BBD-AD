// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from bbd_msgs:msg/BBDBackboneOutput.idl
// generated code does not contain a copyright notice

#ifndef BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__BUILDER_HPP_
#define BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__BUILDER_HPP_

#include "bbd_msgs/msg/detail/bbd_backbone_output__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace bbd_msgs
{

namespace msg
{

namespace builder
{

class Init_BBDBackboneOutput_step
{
public:
  explicit Init_BBDBackboneOutput_step(::bbd_msgs::msg::BBDBackboneOutput & msg)
  : msg_(msg)
  {}
  ::bbd_msgs::msg::BBDBackboneOutput step(::bbd_msgs::msg::BBDBackboneOutput::_step_type arg)
  {
    msg_.step = std::move(arg);
    return std::move(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBackboneOutput msg_;
};

class Init_BBDBackboneOutput_pred_wp
{
public:
  explicit Init_BBDBackboneOutput_pred_wp(::bbd_msgs::msg::BBDBackboneOutput & msg)
  : msg_(msg)
  {}
  Init_BBDBackboneOutput_step pred_wp(::bbd_msgs::msg::BBDBackboneOutput::_pred_wp_type arg)
  {
    msg_.pred_wp = std::move(arg);
    return Init_BBDBackboneOutput_step(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBackboneOutput msg_;
};

class Init_BBDBackboneOutput_command
{
public:
  explicit Init_BBDBackboneOutput_command(::bbd_msgs::msg::BBDBackboneOutput & msg)
  : msg_(msg)
  {}
  Init_BBDBackboneOutput_pred_wp command(::bbd_msgs::msg::BBDBackboneOutput::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_BBDBackboneOutput_pred_wp(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBackboneOutput msg_;
};

class Init_BBDBackboneOutput_target_point
{
public:
  explicit Init_BBDBackboneOutput_target_point(::bbd_msgs::msg::BBDBackboneOutput & msg)
  : msg_(msg)
  {}
  Init_BBDBackboneOutput_command target_point(::bbd_msgs::msg::BBDBackboneOutput::_target_point_type arg)
  {
    msg_.target_point = std::move(arg);
    return Init_BBDBackboneOutput_command(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBackboneOutput msg_;
};

class Init_BBDBackboneOutput_gt_velocity
{
public:
  explicit Init_BBDBackboneOutput_gt_velocity(::bbd_msgs::msg::BBDBackboneOutput & msg)
  : msg_(msg)
  {}
  Init_BBDBackboneOutput_target_point gt_velocity(::bbd_msgs::msg::BBDBackboneOutput::_gt_velocity_type arg)
  {
    msg_.gt_velocity = std::move(arg);
    return Init_BBDBackboneOutput_target_point(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBackboneOutput msg_;
};

class Init_BBDBackboneOutput_speed
{
public:
  explicit Init_BBDBackboneOutput_speed(::bbd_msgs::msg::BBDBackboneOutput & msg)
  : msg_(msg)
  {}
  Init_BBDBackboneOutput_gt_velocity speed(::bbd_msgs::msg::BBDBackboneOutput::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_BBDBackboneOutput_gt_velocity(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBackboneOutput msg_;
};

class Init_BBDBackboneOutput_traj_hidden_state
{
public:
  explicit Init_BBDBackboneOutput_traj_hidden_state(::bbd_msgs::msg::BBDBackboneOutput & msg)
  : msg_(msg)
  {}
  Init_BBDBackboneOutput_speed traj_hidden_state(::bbd_msgs::msg::BBDBackboneOutput::_traj_hidden_state_type arg)
  {
    msg_.traj_hidden_state = std::move(arg);
    return Init_BBDBackboneOutput_speed(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBackboneOutput msg_;
};

class Init_BBDBackboneOutput_measurement_feature
{
public:
  explicit Init_BBDBackboneOutput_measurement_feature(::bbd_msgs::msg::BBDBackboneOutput & msg)
  : msg_(msg)
  {}
  Init_BBDBackboneOutput_traj_hidden_state measurement_feature(::bbd_msgs::msg::BBDBackboneOutput::_measurement_feature_type arg)
  {
    msg_.measurement_feature = std::move(arg);
    return Init_BBDBackboneOutput_traj_hidden_state(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBackboneOutput msg_;
};

class Init_BBDBackboneOutput_cnn_feature
{
public:
  Init_BBDBackboneOutput_cnn_feature()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BBDBackboneOutput_measurement_feature cnn_feature(::bbd_msgs::msg::BBDBackboneOutput::_cnn_feature_type arg)
  {
    msg_.cnn_feature = std::move(arg);
    return Init_BBDBackboneOutput_measurement_feature(msg_);
  }

private:
  ::bbd_msgs::msg::BBDBackboneOutput msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::bbd_msgs::msg::BBDBackboneOutput>()
{
  return bbd_msgs::msg::builder::Init_BBDBackboneOutput_cnn_feature();
}

}  // namespace bbd_msgs

#endif  // BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__BUILDER_HPP_
