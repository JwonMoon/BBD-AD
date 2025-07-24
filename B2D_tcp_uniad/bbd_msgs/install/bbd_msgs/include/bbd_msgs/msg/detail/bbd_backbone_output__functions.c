// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from bbd_msgs:msg/BBDBackboneOutput.idl
// generated code does not contain a copyright notice
#include "bbd_msgs/msg/detail/bbd_backbone_output__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `cnn_feature`
// Member `measurement_feature`
// Member `traj_hidden_state`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
bbd_msgs__msg__BBDBackboneOutput__init(bbd_msgs__msg__BBDBackboneOutput * msg)
{
  if (!msg) {
    return false;
  }
  // cnn_feature
  if (!rosidl_runtime_c__float__Sequence__init(&msg->cnn_feature, 0)) {
    bbd_msgs__msg__BBDBackboneOutput__fini(msg);
    return false;
  }
  // measurement_feature
  if (!rosidl_runtime_c__float__Sequence__init(&msg->measurement_feature, 0)) {
    bbd_msgs__msg__BBDBackboneOutput__fini(msg);
    return false;
  }
  // traj_hidden_state
  if (!rosidl_runtime_c__float__Sequence__init(&msg->traj_hidden_state, 0)) {
    bbd_msgs__msg__BBDBackboneOutput__fini(msg);
    return false;
  }
  // speed
  // gt_velocity
  // target_point
  // command
  // pred_wp
  // step
  return true;
}

void
bbd_msgs__msg__BBDBackboneOutput__fini(bbd_msgs__msg__BBDBackboneOutput * msg)
{
  if (!msg) {
    return;
  }
  // cnn_feature
  rosidl_runtime_c__float__Sequence__fini(&msg->cnn_feature);
  // measurement_feature
  rosidl_runtime_c__float__Sequence__fini(&msg->measurement_feature);
  // traj_hidden_state
  rosidl_runtime_c__float__Sequence__fini(&msg->traj_hidden_state);
  // speed
  // gt_velocity
  // target_point
  // command
  // pred_wp
  // step
}

bool
bbd_msgs__msg__BBDBackboneOutput__are_equal(const bbd_msgs__msg__BBDBackboneOutput * lhs, const bbd_msgs__msg__BBDBackboneOutput * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cnn_feature
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->cnn_feature), &(rhs->cnn_feature)))
  {
    return false;
  }
  // measurement_feature
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->measurement_feature), &(rhs->measurement_feature)))
  {
    return false;
  }
  // traj_hidden_state
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->traj_hidden_state), &(rhs->traj_hidden_state)))
  {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // gt_velocity
  if (lhs->gt_velocity != rhs->gt_velocity) {
    return false;
  }
  // target_point
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->target_point[i] != rhs->target_point[i]) {
      return false;
    }
  }
  // command
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->command[i] != rhs->command[i]) {
      return false;
    }
  }
  // pred_wp
  for (size_t i = 0; i < 8; ++i) {
    if (lhs->pred_wp[i] != rhs->pred_wp[i]) {
      return false;
    }
  }
  // step
  if (lhs->step != rhs->step) {
    return false;
  }
  return true;
}

bool
bbd_msgs__msg__BBDBackboneOutput__copy(
  const bbd_msgs__msg__BBDBackboneOutput * input,
  bbd_msgs__msg__BBDBackboneOutput * output)
{
  if (!input || !output) {
    return false;
  }
  // cnn_feature
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->cnn_feature), &(output->cnn_feature)))
  {
    return false;
  }
  // measurement_feature
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->measurement_feature), &(output->measurement_feature)))
  {
    return false;
  }
  // traj_hidden_state
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->traj_hidden_state), &(output->traj_hidden_state)))
  {
    return false;
  }
  // speed
  output->speed = input->speed;
  // gt_velocity
  output->gt_velocity = input->gt_velocity;
  // target_point
  for (size_t i = 0; i < 2; ++i) {
    output->target_point[i] = input->target_point[i];
  }
  // command
  for (size_t i = 0; i < 6; ++i) {
    output->command[i] = input->command[i];
  }
  // pred_wp
  for (size_t i = 0; i < 8; ++i) {
    output->pred_wp[i] = input->pred_wp[i];
  }
  // step
  output->step = input->step;
  return true;
}

bbd_msgs__msg__BBDBackboneOutput *
bbd_msgs__msg__BBDBackboneOutput__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bbd_msgs__msg__BBDBackboneOutput * msg = (bbd_msgs__msg__BBDBackboneOutput *)allocator.allocate(sizeof(bbd_msgs__msg__BBDBackboneOutput), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bbd_msgs__msg__BBDBackboneOutput));
  bool success = bbd_msgs__msg__BBDBackboneOutput__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bbd_msgs__msg__BBDBackboneOutput__destroy(bbd_msgs__msg__BBDBackboneOutput * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bbd_msgs__msg__BBDBackboneOutput__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bbd_msgs__msg__BBDBackboneOutput__Sequence__init(bbd_msgs__msg__BBDBackboneOutput__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bbd_msgs__msg__BBDBackboneOutput * data = NULL;

  if (size) {
    data = (bbd_msgs__msg__BBDBackboneOutput *)allocator.zero_allocate(size, sizeof(bbd_msgs__msg__BBDBackboneOutput), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bbd_msgs__msg__BBDBackboneOutput__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bbd_msgs__msg__BBDBackboneOutput__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
bbd_msgs__msg__BBDBackboneOutput__Sequence__fini(bbd_msgs__msg__BBDBackboneOutput__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      bbd_msgs__msg__BBDBackboneOutput__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

bbd_msgs__msg__BBDBackboneOutput__Sequence *
bbd_msgs__msg__BBDBackboneOutput__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bbd_msgs__msg__BBDBackboneOutput__Sequence * array = (bbd_msgs__msg__BBDBackboneOutput__Sequence *)allocator.allocate(sizeof(bbd_msgs__msg__BBDBackboneOutput__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bbd_msgs__msg__BBDBackboneOutput__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bbd_msgs__msg__BBDBackboneOutput__Sequence__destroy(bbd_msgs__msg__BBDBackboneOutput__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bbd_msgs__msg__BBDBackboneOutput__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bbd_msgs__msg__BBDBackboneOutput__Sequence__are_equal(const bbd_msgs__msg__BBDBackboneOutput__Sequence * lhs, const bbd_msgs__msg__BBDBackboneOutput__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bbd_msgs__msg__BBDBackboneOutput__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bbd_msgs__msg__BBDBackboneOutput__Sequence__copy(
  const bbd_msgs__msg__BBDBackboneOutput__Sequence * input,
  bbd_msgs__msg__BBDBackboneOutput__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bbd_msgs__msg__BBDBackboneOutput);
    bbd_msgs__msg__BBDBackboneOutput * data =
      (bbd_msgs__msg__BBDBackboneOutput *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bbd_msgs__msg__BBDBackboneOutput__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          bbd_msgs__msg__BBDBackboneOutput__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!bbd_msgs__msg__BBDBackboneOutput__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
