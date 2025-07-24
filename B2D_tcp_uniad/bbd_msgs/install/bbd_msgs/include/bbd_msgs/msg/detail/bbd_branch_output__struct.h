// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from bbd_msgs:msg/BBDBranchOutput.idl
// generated code does not contain a copyright notice

#ifndef BBD_MSGS__MSG__DETAIL__BBD_BRANCH_OUTPUT__STRUCT_H_
#define BBD_MSGS__MSG__DETAIL__BBD_BRANCH_OUTPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

// Struct defined in msg/BBDBranchOutput in the package bbd_msgs.
typedef struct bbd_msgs__msg__BBDBranchOutput
{
  std_msgs__msg__Header header;
  float throttle;
  float steer;
  float brake;
  bool hand_brake;
  bool reverse;
  int32_t gear;
  bool manual_gear_shift;
  int32_t step;
} bbd_msgs__msg__BBDBranchOutput;

// Struct for a sequence of bbd_msgs__msg__BBDBranchOutput.
typedef struct bbd_msgs__msg__BBDBranchOutput__Sequence
{
  bbd_msgs__msg__BBDBranchOutput * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bbd_msgs__msg__BBDBranchOutput__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BBD_MSGS__MSG__DETAIL__BBD_BRANCH_OUTPUT__STRUCT_H_
