// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from bbd_msgs:msg/TickTrigger.idl
// generated code does not contain a copyright notice

#ifndef BBD_MSGS__MSG__DETAIL__TICK_TRIGGER__STRUCT_HPP_
#define BBD_MSGS__MSG__DETAIL__TICK_TRIGGER__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__bbd_msgs__msg__TickTrigger __attribute__((deprecated))
#else
# define DEPRECATED__bbd_msgs__msg__TickTrigger __declspec(deprecated)
#endif

namespace bbd_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TickTrigger_
{
  using Type = TickTrigger_<ContainerAllocator>;

  explicit TickTrigger_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->step = 0l;
      this->trigger = false;
    }
  }

  explicit TickTrigger_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->step = 0l;
      this->trigger = false;
    }
  }

  // field types and members
  using _step_type =
    int32_t;
  _step_type step;
  using _trigger_type =
    bool;
  _trigger_type trigger;

  // setters for named parameter idiom
  Type & set__step(
    const int32_t & _arg)
  {
    this->step = _arg;
    return *this;
  }
  Type & set__trigger(
    const bool & _arg)
  {
    this->trigger = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    bbd_msgs::msg::TickTrigger_<ContainerAllocator> *;
  using ConstRawPtr =
    const bbd_msgs::msg::TickTrigger_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<bbd_msgs::msg::TickTrigger_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<bbd_msgs::msg::TickTrigger_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      bbd_msgs::msg::TickTrigger_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<bbd_msgs::msg::TickTrigger_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      bbd_msgs::msg::TickTrigger_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<bbd_msgs::msg::TickTrigger_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<bbd_msgs::msg::TickTrigger_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<bbd_msgs::msg::TickTrigger_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__bbd_msgs__msg__TickTrigger
    std::shared_ptr<bbd_msgs::msg::TickTrigger_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__bbd_msgs__msg__TickTrigger
    std::shared_ptr<bbd_msgs::msg::TickTrigger_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TickTrigger_ & other) const
  {
    if (this->step != other.step) {
      return false;
    }
    if (this->trigger != other.trigger) {
      return false;
    }
    return true;
  }
  bool operator!=(const TickTrigger_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TickTrigger_

// alias to use template instance with default allocator
using TickTrigger =
  bbd_msgs::msg::TickTrigger_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace bbd_msgs

#endif  // BBD_MSGS__MSG__DETAIL__TICK_TRIGGER__STRUCT_HPP_
