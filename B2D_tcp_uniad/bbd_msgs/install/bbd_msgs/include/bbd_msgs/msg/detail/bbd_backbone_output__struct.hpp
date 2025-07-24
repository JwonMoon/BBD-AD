// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from bbd_msgs:msg/BBDBackboneOutput.idl
// generated code does not contain a copyright notice

#ifndef BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__STRUCT_HPP_
#define BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__bbd_msgs__msg__BBDBackboneOutput __attribute__((deprecated))
#else
# define DEPRECATED__bbd_msgs__msg__BBDBackboneOutput __declspec(deprecated)
#endif

namespace bbd_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BBDBackboneOutput_
{
  using Type = BBDBackboneOutput_<ContainerAllocator>;

  explicit BBDBackboneOutput_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0f;
      this->gt_velocity = 0.0f;
      std::fill<typename std::array<float, 2>::iterator, float>(this->target_point.begin(), this->target_point.end(), 0.0f);
      std::fill<typename std::array<float, 6>::iterator, float>(this->command.begin(), this->command.end(), 0.0f);
      std::fill<typename std::array<float, 8>::iterator, float>(this->pred_wp.begin(), this->pred_wp.end(), 0.0f);
      this->step = 0l;
    }
  }

  explicit BBDBackboneOutput_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : target_point(_alloc),
    command(_alloc),
    pred_wp(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->speed = 0.0f;
      this->gt_velocity = 0.0f;
      std::fill<typename std::array<float, 2>::iterator, float>(this->target_point.begin(), this->target_point.end(), 0.0f);
      std::fill<typename std::array<float, 6>::iterator, float>(this->command.begin(), this->command.end(), 0.0f);
      std::fill<typename std::array<float, 8>::iterator, float>(this->pred_wp.begin(), this->pred_wp.end(), 0.0f);
      this->step = 0l;
    }
  }

  // field types and members
  using _cnn_feature_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _cnn_feature_type cnn_feature;
  using _measurement_feature_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _measurement_feature_type measurement_feature;
  using _traj_hidden_state_type =
    std::vector<float, typename ContainerAllocator::template rebind<float>::other>;
  _traj_hidden_state_type traj_hidden_state;
  using _speed_type =
    float;
  _speed_type speed;
  using _gt_velocity_type =
    float;
  _gt_velocity_type gt_velocity;
  using _target_point_type =
    std::array<float, 2>;
  _target_point_type target_point;
  using _command_type =
    std::array<float, 6>;
  _command_type command;
  using _pred_wp_type =
    std::array<float, 8>;
  _pred_wp_type pred_wp;
  using _step_type =
    int32_t;
  _step_type step;

  // setters for named parameter idiom
  Type & set__cnn_feature(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->cnn_feature = _arg;
    return *this;
  }
  Type & set__measurement_feature(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->measurement_feature = _arg;
    return *this;
  }
  Type & set__traj_hidden_state(
    const std::vector<float, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->traj_hidden_state = _arg;
    return *this;
  }
  Type & set__speed(
    const float & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__gt_velocity(
    const float & _arg)
  {
    this->gt_velocity = _arg;
    return *this;
  }
  Type & set__target_point(
    const std::array<float, 2> & _arg)
  {
    this->target_point = _arg;
    return *this;
  }
  Type & set__command(
    const std::array<float, 6> & _arg)
  {
    this->command = _arg;
    return *this;
  }
  Type & set__pred_wp(
    const std::array<float, 8> & _arg)
  {
    this->pred_wp = _arg;
    return *this;
  }
  Type & set__step(
    const int32_t & _arg)
  {
    this->step = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    bbd_msgs::msg::BBDBackboneOutput_<ContainerAllocator> *;
  using ConstRawPtr =
    const bbd_msgs::msg::BBDBackboneOutput_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<bbd_msgs::msg::BBDBackboneOutput_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<bbd_msgs::msg::BBDBackboneOutput_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      bbd_msgs::msg::BBDBackboneOutput_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<bbd_msgs::msg::BBDBackboneOutput_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      bbd_msgs::msg::BBDBackboneOutput_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<bbd_msgs::msg::BBDBackboneOutput_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<bbd_msgs::msg::BBDBackboneOutput_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<bbd_msgs::msg::BBDBackboneOutput_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__bbd_msgs__msg__BBDBackboneOutput
    std::shared_ptr<bbd_msgs::msg::BBDBackboneOutput_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__bbd_msgs__msg__BBDBackboneOutput
    std::shared_ptr<bbd_msgs::msg::BBDBackboneOutput_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BBDBackboneOutput_ & other) const
  {
    if (this->cnn_feature != other.cnn_feature) {
      return false;
    }
    if (this->measurement_feature != other.measurement_feature) {
      return false;
    }
    if (this->traj_hidden_state != other.traj_hidden_state) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->gt_velocity != other.gt_velocity) {
      return false;
    }
    if (this->target_point != other.target_point) {
      return false;
    }
    if (this->command != other.command) {
      return false;
    }
    if (this->pred_wp != other.pred_wp) {
      return false;
    }
    if (this->step != other.step) {
      return false;
    }
    return true;
  }
  bool operator!=(const BBDBackboneOutput_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BBDBackboneOutput_

// alias to use template instance with default allocator
using BBDBackboneOutput =
  bbd_msgs::msg::BBDBackboneOutput_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace bbd_msgs

#endif  // BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__STRUCT_HPP_
