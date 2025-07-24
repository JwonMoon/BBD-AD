// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from bbd_msgs:msg/BBDBackboneOutput.idl
// generated code does not contain a copyright notice

#ifndef BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__STRUCT_H_
#define BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'cnn_feature'
// Member 'measurement_feature'
// Member 'traj_hidden_state'
#include "rosidl_runtime_c/primitives_sequence.h"

// Struct defined in msg/BBDBackboneOutput in the package bbd_msgs.
typedef struct bbd_msgs__msg__BBDBackboneOutput
{
  rosidl_runtime_c__float__Sequence cnn_feature;
  rosidl_runtime_c__float__Sequence measurement_feature;
  rosidl_runtime_c__float__Sequence traj_hidden_state;
  float speed;
  float gt_velocity;
  float target_point[2];
  float command[6];
  float pred_wp[8];
  int32_t step;
} bbd_msgs__msg__BBDBackboneOutput;

// Struct for a sequence of bbd_msgs__msg__BBDBackboneOutput.
typedef struct bbd_msgs__msg__BBDBackboneOutput__Sequence
{
  bbd_msgs__msg__BBDBackboneOutput * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} bbd_msgs__msg__BBDBackboneOutput__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__STRUCT_H_
