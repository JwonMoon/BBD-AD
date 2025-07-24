// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from bbd_msgs:msg/TickTrigger.idl
// generated code does not contain a copyright notice

#ifndef BBD_MSGS__MSG__DETAIL__TICK_TRIGGER__STRUCT_H_
#define BBD_MSGS__MSG__DETAIL__TICK_TRIGGER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/TickTrigger in the package bbd_msgs.
typedef struct bbd_msgs__msg__TickTrigger
{
  int32_t step;
  bool trigger;
} bbd_msgs__msg__TickTrigger;

// Struct for a sequence of bbd_msgs__msg__TickTrigger.
typedef struct bbd_msgs__msg__TickTrigger__Sequence
{
  bbd_msgs__msg__TickTrigger * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bbd_msgs__msg__TickTrigger__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BBD_MSGS__MSG__DETAIL__TICK_TRIGGER__STRUCT_H_
