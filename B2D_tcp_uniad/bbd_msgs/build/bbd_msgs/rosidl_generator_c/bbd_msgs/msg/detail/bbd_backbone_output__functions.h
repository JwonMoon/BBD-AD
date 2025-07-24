// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from bbd_msgs:msg/BBDBackboneOutput.idl
// generated code does not contain a copyright notice

#ifndef BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__FUNCTIONS_H_
#define BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "bbd_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "bbd_msgs/msg/detail/bbd_backbone_output__struct.h"

/// Initialize msg/BBDBackboneOutput message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * bbd_msgs__msg__BBDBackboneOutput
 * )) before or use
 * bbd_msgs__msg__BBDBackboneOutput__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_bbd_msgs
bool
bbd_msgs__msg__BBDBackboneOutput__init(bbd_msgs__msg__BBDBackboneOutput * msg);

/// Finalize msg/BBDBackboneOutput message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bbd_msgs
void
bbd_msgs__msg__BBDBackboneOutput__fini(bbd_msgs__msg__BBDBackboneOutput * msg);

/// Create msg/BBDBackboneOutput message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * bbd_msgs__msg__BBDBackboneOutput__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_bbd_msgs
bbd_msgs__msg__BBDBackboneOutput *
bbd_msgs__msg__BBDBackboneOutput__create();

/// Destroy msg/BBDBackboneOutput message.
/**
 * It calls
 * bbd_msgs__msg__BBDBackboneOutput__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bbd_msgs
void
bbd_msgs__msg__BBDBackboneOutput__destroy(bbd_msgs__msg__BBDBackboneOutput * msg);

/// Check for msg/BBDBackboneOutput message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_bbd_msgs
bool
bbd_msgs__msg__BBDBackboneOutput__are_equal(const bbd_msgs__msg__BBDBackboneOutput * lhs, const bbd_msgs__msg__BBDBackboneOutput * rhs);

/// Copy a msg/BBDBackboneOutput message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_bbd_msgs
bool
bbd_msgs__msg__BBDBackboneOutput__copy(
  const bbd_msgs__msg__BBDBackboneOutput * input,
  bbd_msgs__msg__BBDBackboneOutput * output);

/// Initialize array of msg/BBDBackboneOutput messages.
/**
 * It allocates the memory for the number of elements and calls
 * bbd_msgs__msg__BBDBackboneOutput__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_bbd_msgs
bool
bbd_msgs__msg__BBDBackboneOutput__Sequence__init(bbd_msgs__msg__BBDBackboneOutput__Sequence * array, size_t size);

/// Finalize array of msg/BBDBackboneOutput messages.
/**
 * It calls
 * bbd_msgs__msg__BBDBackboneOutput__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bbd_msgs
void
bbd_msgs__msg__BBDBackboneOutput__Sequence__fini(bbd_msgs__msg__BBDBackboneOutput__Sequence * array);

/// Create array of msg/BBDBackboneOutput messages.
/**
 * It allocates the memory for the array and calls
 * bbd_msgs__msg__BBDBackboneOutput__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_bbd_msgs
bbd_msgs__msg__BBDBackboneOutput__Sequence *
bbd_msgs__msg__BBDBackboneOutput__Sequence__create(size_t size);

/// Destroy array of msg/BBDBackboneOutput messages.
/**
 * It calls
 * bbd_msgs__msg__BBDBackboneOutput__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_bbd_msgs
void
bbd_msgs__msg__BBDBackboneOutput__Sequence__destroy(bbd_msgs__msg__BBDBackboneOutput__Sequence * array);

/// Check for msg/BBDBackboneOutput message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_bbd_msgs
bool
bbd_msgs__msg__BBDBackboneOutput__Sequence__are_equal(const bbd_msgs__msg__BBDBackboneOutput__Sequence * lhs, const bbd_msgs__msg__BBDBackboneOutput__Sequence * rhs);

/// Copy an array of msg/BBDBackboneOutput messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_bbd_msgs
bool
bbd_msgs__msg__BBDBackboneOutput__Sequence__copy(
  const bbd_msgs__msg__BBDBackboneOutput__Sequence * input,
  bbd_msgs__msg__BBDBackboneOutput__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // BBD_MSGS__MSG__DETAIL__BBD_BACKBONE_OUTPUT__FUNCTIONS_H_
