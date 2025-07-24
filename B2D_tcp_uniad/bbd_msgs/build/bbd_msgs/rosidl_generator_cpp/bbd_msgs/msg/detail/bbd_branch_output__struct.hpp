// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from bbd_msgs:msg/BBDBranchOutput.idl
// generated code does not contain a copyright notice

#ifndef BBD_MSGS__MSG__DETAIL__BBD_BRANCH_OUTPUT__STRUCT_HPP_
#define BBD_MSGS__MSG__DETAIL__BBD_BRANCH_OUTPUT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__bbd_msgs__msg__BBDBranchOutput __attribute__((deprecated))
#else
# define DEPRECATED__bbd_msgs__msg__BBDBranchOutput __declspec(deprecated)
#endif

namespace bbd_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BBDBranchOutput_
{
  using Type = BBDBranchOutput_<ContainerAllocator>;

  explicit BBDBranchOutput_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->throttle = 0.0f;
      this->steer = 0.0f;
      this->brake = 0.0f;
      this->hand_brake = false;
      this->reverse = false;
      this->gear = 0l;
      this->manual_gear_shift = false;
      this->step = 0l;
    }
  }

  explicit BBDBranchOutput_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->throttle = 0.0f;
      this->steer = 0.0f;
      this->brake = 0.0f;
      this->hand_brake = false;
      this->reverse = false;
      this->gear = 0l;
      this->manual_gear_shift = false;
      this->step = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _throttle_type =
    float;
  _throttle_type throttle;
  using _steer_type =
    float;
  _steer_type steer;
  using _brake_type =
    float;
  _brake_type brake;
  using _hand_brake_type =
    bool;
  _hand_brake_type hand_brake;
  using _reverse_type =
    bool;
  _reverse_type reverse;
  using _gear_type =
    int32_t;
  _gear_type gear;
  using _manual_gear_shift_type =
    bool;
  _manual_gear_shift_type manual_gear_shift;
  using _step_type =
    int32_t;
  _step_type step;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__throttle(
    const float & _arg)
  {
    this->throttle = _arg;
    return *this;
  }
  Type & set__steer(
    const float & _arg)
  {
    this->steer = _arg;
    return *this;
  }
  Type & set__brake(
    const float & _arg)
  {
    this->brake = _arg;
    return *this;
  }
  Type & set__hand_brake(
    const bool & _arg)
  {
    this->hand_brake = _arg;
    return *this;
  }
  Type & set__reverse(
    const bool & _arg)
  {
    this->reverse = _arg;
    return *this;
  }
  Type & set__gear(
    const int32_t & _arg)
  {
    this->gear = _arg;
    return *this;
  }
  Type & set__manual_gear_shift(
    const bool & _arg)
  {
    this->manual_gear_shift = _arg;
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
    bbd_msgs::msg::BBDBranchOutput_<ContainerAllocator> *;
  using ConstRawPtr =
    const bbd_msgs::msg::BBDBranchOutput_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<bbd_msgs::msg::BBDBranchOutput_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<bbd_msgs::msg::BBDBranchOutput_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      bbd_msgs::msg::BBDBranchOutput_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<bbd_msgs::msg::BBDBranchOutput_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      bbd_msgs::msg::BBDBranchOutput_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<bbd_msgs::msg::BBDBranchOutput_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<bbd_msgs::msg::BBDBranchOutput_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<bbd_msgs::msg::BBDBranchOutput_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__bbd_msgs__msg__BBDBranchOutput
    std::shared_ptr<bbd_msgs::msg::BBDBranchOutput_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__bbd_msgs__msg__BBDBranchOutput
    std::shared_ptr<bbd_msgs::msg::BBDBranchOutput_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BBDBranchOutput_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->throttle != other.throttle) {
      return false;
    }
    if (this->steer != other.steer) {
      return false;
    }
    if (this->brake != other.brake) {
      return false;
    }
    if (this->hand_brake != other.hand_brake) {
      return false;
    }
    if (this->reverse != other.reverse) {
      return false;
    }
    if (this->gear != other.gear) {
      return false;
    }
    if (this->manual_gear_shift != other.manual_gear_shift) {
      return false;
    }
    if (this->step != other.step) {
      return false;
    }
    return true;
  }
  bool operator!=(const BBDBranchOutput_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BBDBranchOutput_

// alias to use template instance with default allocator
using BBDBranchOutput =
  bbd_msgs::msg::BBDBranchOutput_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace bbd_msgs

#endif  // BBD_MSGS__MSG__DETAIL__BBD_BRANCH_OUTPUT__STRUCT_HPP_
