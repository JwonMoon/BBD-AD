// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from bbd_msgs:msg/BBDBranchOutput.idl
// generated code does not contain a copyright notice
#include "bbd_msgs/msg/detail/bbd_branch_output__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
bbd_msgs__msg__BBDBranchOutput__init(bbd_msgs__msg__BBDBranchOutput * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    bbd_msgs__msg__BBDBranchOutput__fini(msg);
    return false;
  }
  // throttle
  // steer
  // brake
  // hand_brake
  // reverse
  // gear
  // manual_gear_shift
  // step
  return true;
}

void
bbd_msgs__msg__BBDBranchOutput__fini(bbd_msgs__msg__BBDBranchOutput * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // throttle
  // steer
  // brake
  // hand_brake
  // reverse
  // gear
  // manual_gear_shift
  // step
}

bool
bbd_msgs__msg__BBDBranchOutput__are_equal(const bbd_msgs__msg__BBDBranchOutput * lhs, const bbd_msgs__msg__BBDBranchOutput * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // throttle
  if (lhs->throttle != rhs->throttle) {
    return false;
  }
  // steer
  if (lhs->steer != rhs->steer) {
    return false;
  }
  // brake
  if (lhs->brake != rhs->brake) {
    return false;
  }
  // hand_brake
  if (lhs->hand_brake != rhs->hand_brake) {
    return false;
  }
  // reverse
  if (lhs->reverse != rhs->reverse) {
    return false;
  }
  // gear
  if (lhs->gear != rhs->gear) {
    return false;
  }
  // manual_gear_shift
  if (lhs->manual_gear_shift != rhs->manual_gear_shift) {
    return false;
  }
  // step
  if (lhs->step != rhs->step) {
    return false;
  }
  return true;
}

bool
bbd_msgs__msg__BBDBranchOutput__copy(
  const bbd_msgs__msg__BBDBranchOutput * input,
  bbd_msgs__msg__BBDBranchOutput * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // throttle
  output->throttle = input->throttle;
  // steer
  output->steer = input->steer;
  // brake
  output->brake = input->brake;
  // hand_brake
  output->hand_brake = input->hand_brake;
  // reverse
  output->reverse = input->reverse;
  // gear
  output->gear = input->gear;
  // manual_gear_shift
  output->manual_gear_shift = input->manual_gear_shift;
  // step
  output->step = input->step;
  return true;
}

bbd_msgs__msg__BBDBranchOutput *
bbd_msgs__msg__BBDBranchOutput__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bbd_msgs__msg__BBDBranchOutput * msg = (bbd_msgs__msg__BBDBranchOutput *)allocator.allocate(sizeof(bbd_msgs__msg__BBDBranchOutput), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bbd_msgs__msg__BBDBranchOutput));
  bool success = bbd_msgs__msg__BBDBranchOutput__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bbd_msgs__msg__BBDBranchOutput__destroy(bbd_msgs__msg__BBDBranchOutput * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bbd_msgs__msg__BBDBranchOutput__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bbd_msgs__msg__BBDBranchOutput__Sequence__init(bbd_msgs__msg__BBDBranchOutput__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bbd_msgs__msg__BBDBranchOutput * data = NULL;

  if (size) {
    data = (bbd_msgs__msg__BBDBranchOutput *)allocator.zero_allocate(size, sizeof(bbd_msgs__msg__BBDBranchOutput), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bbd_msgs__msg__BBDBranchOutput__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bbd_msgs__msg__BBDBranchOutput__fini(&data[i - 1]);
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
bbd_msgs__msg__BBDBranchOutput__Sequence__fini(bbd_msgs__msg__BBDBranchOutput__Sequence * array)
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
      bbd_msgs__msg__BBDBranchOutput__fini(&array->data[i]);
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

bbd_msgs__msg__BBDBranchOutput__Sequence *
bbd_msgs__msg__BBDBranchOutput__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bbd_msgs__msg__BBDBranchOutput__Sequence * array = (bbd_msgs__msg__BBDBranchOutput__Sequence *)allocator.allocate(sizeof(bbd_msgs__msg__BBDBranchOutput__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bbd_msgs__msg__BBDBranchOutput__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bbd_msgs__msg__BBDBranchOutput__Sequence__destroy(bbd_msgs__msg__BBDBranchOutput__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bbd_msgs__msg__BBDBranchOutput__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bbd_msgs__msg__BBDBranchOutput__Sequence__are_equal(const bbd_msgs__msg__BBDBranchOutput__Sequence * lhs, const bbd_msgs__msg__BBDBranchOutput__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bbd_msgs__msg__BBDBranchOutput__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bbd_msgs__msg__BBDBranchOutput__Sequence__copy(
  const bbd_msgs__msg__BBDBranchOutput__Sequence * input,
  bbd_msgs__msg__BBDBranchOutput__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bbd_msgs__msg__BBDBranchOutput);
    bbd_msgs__msg__BBDBranchOutput * data =
      (bbd_msgs__msg__BBDBranchOutput *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bbd_msgs__msg__BBDBranchOutput__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          bbd_msgs__msg__BBDBranchOutput__fini(&data[i]);
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
    if (!bbd_msgs__msg__BBDBranchOutput__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
