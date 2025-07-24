// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from bbd_msgs:msg/TickTrigger.idl
// generated code does not contain a copyright notice
#include "bbd_msgs/msg/detail/tick_trigger__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
bbd_msgs__msg__TickTrigger__init(bbd_msgs__msg__TickTrigger * msg)
{
  if (!msg) {
    return false;
  }
  // step
  // trigger
  return true;
}

void
bbd_msgs__msg__TickTrigger__fini(bbd_msgs__msg__TickTrigger * msg)
{
  if (!msg) {
    return;
  }
  // step
  // trigger
}

bool
bbd_msgs__msg__TickTrigger__are_equal(const bbd_msgs__msg__TickTrigger * lhs, const bbd_msgs__msg__TickTrigger * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // step
  if (lhs->step != rhs->step) {
    return false;
  }
  // trigger
  if (lhs->trigger != rhs->trigger) {
    return false;
  }
  return true;
}

bool
bbd_msgs__msg__TickTrigger__copy(
  const bbd_msgs__msg__TickTrigger * input,
  bbd_msgs__msg__TickTrigger * output)
{
  if (!input || !output) {
    return false;
  }
  // step
  output->step = input->step;
  // trigger
  output->trigger = input->trigger;
  return true;
}

bbd_msgs__msg__TickTrigger *
bbd_msgs__msg__TickTrigger__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bbd_msgs__msg__TickTrigger * msg = (bbd_msgs__msg__TickTrigger *)allocator.allocate(sizeof(bbd_msgs__msg__TickTrigger), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(bbd_msgs__msg__TickTrigger));
  bool success = bbd_msgs__msg__TickTrigger__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
bbd_msgs__msg__TickTrigger__destroy(bbd_msgs__msg__TickTrigger * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    bbd_msgs__msg__TickTrigger__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
bbd_msgs__msg__TickTrigger__Sequence__init(bbd_msgs__msg__TickTrigger__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bbd_msgs__msg__TickTrigger * data = NULL;

  if (size) {
    data = (bbd_msgs__msg__TickTrigger *)allocator.zero_allocate(size, sizeof(bbd_msgs__msg__TickTrigger), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = bbd_msgs__msg__TickTrigger__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        bbd_msgs__msg__TickTrigger__fini(&data[i - 1]);
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
bbd_msgs__msg__TickTrigger__Sequence__fini(bbd_msgs__msg__TickTrigger__Sequence * array)
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
      bbd_msgs__msg__TickTrigger__fini(&array->data[i]);
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

bbd_msgs__msg__TickTrigger__Sequence *
bbd_msgs__msg__TickTrigger__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  bbd_msgs__msg__TickTrigger__Sequence * array = (bbd_msgs__msg__TickTrigger__Sequence *)allocator.allocate(sizeof(bbd_msgs__msg__TickTrigger__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = bbd_msgs__msg__TickTrigger__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
bbd_msgs__msg__TickTrigger__Sequence__destroy(bbd_msgs__msg__TickTrigger__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    bbd_msgs__msg__TickTrigger__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
bbd_msgs__msg__TickTrigger__Sequence__are_equal(const bbd_msgs__msg__TickTrigger__Sequence * lhs, const bbd_msgs__msg__TickTrigger__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!bbd_msgs__msg__TickTrigger__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
bbd_msgs__msg__TickTrigger__Sequence__copy(
  const bbd_msgs__msg__TickTrigger__Sequence * input,
  bbd_msgs__msg__TickTrigger__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(bbd_msgs__msg__TickTrigger);
    bbd_msgs__msg__TickTrigger * data =
      (bbd_msgs__msg__TickTrigger *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!bbd_msgs__msg__TickTrigger__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          bbd_msgs__msg__TickTrigger__fini(&data[i]);
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
    if (!bbd_msgs__msg__TickTrigger__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
