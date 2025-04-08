// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from champ_msgs:msg/Joints.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__JOINTS__FUNCTIONS_H_
#define CHAMP_MSGS__MSG__DETAIL__JOINTS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "champ_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "champ_msgs/msg/detail/joints__struct.h"

/// Initialize msg/Joints message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * champ_msgs__msg__Joints
 * )) before or use
 * champ_msgs__msg__Joints__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_champ_msgs
bool
champ_msgs__msg__Joints__init(champ_msgs__msg__Joints * msg);

/// Finalize msg/Joints message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_champ_msgs
void
champ_msgs__msg__Joints__fini(champ_msgs__msg__Joints * msg);

/// Create msg/Joints message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * champ_msgs__msg__Joints__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_champ_msgs
champ_msgs__msg__Joints *
champ_msgs__msg__Joints__create();

/// Destroy msg/Joints message.
/**
 * It calls
 * champ_msgs__msg__Joints__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_champ_msgs
void
champ_msgs__msg__Joints__destroy(champ_msgs__msg__Joints * msg);

/// Check for msg/Joints message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_champ_msgs
bool
champ_msgs__msg__Joints__are_equal(const champ_msgs__msg__Joints * lhs, const champ_msgs__msg__Joints * rhs);

/// Copy a msg/Joints message.
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
ROSIDL_GENERATOR_C_PUBLIC_champ_msgs
bool
champ_msgs__msg__Joints__copy(
  const champ_msgs__msg__Joints * input,
  champ_msgs__msg__Joints * output);

/// Initialize array of msg/Joints messages.
/**
 * It allocates the memory for the number of elements and calls
 * champ_msgs__msg__Joints__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_champ_msgs
bool
champ_msgs__msg__Joints__Sequence__init(champ_msgs__msg__Joints__Sequence * array, size_t size);

/// Finalize array of msg/Joints messages.
/**
 * It calls
 * champ_msgs__msg__Joints__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_champ_msgs
void
champ_msgs__msg__Joints__Sequence__fini(champ_msgs__msg__Joints__Sequence * array);

/// Create array of msg/Joints messages.
/**
 * It allocates the memory for the array and calls
 * champ_msgs__msg__Joints__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_champ_msgs
champ_msgs__msg__Joints__Sequence *
champ_msgs__msg__Joints__Sequence__create(size_t size);

/// Destroy array of msg/Joints messages.
/**
 * It calls
 * champ_msgs__msg__Joints__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_champ_msgs
void
champ_msgs__msg__Joints__Sequence__destroy(champ_msgs__msg__Joints__Sequence * array);

/// Check for msg/Joints message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_champ_msgs
bool
champ_msgs__msg__Joints__Sequence__are_equal(const champ_msgs__msg__Joints__Sequence * lhs, const champ_msgs__msg__Joints__Sequence * rhs);

/// Copy an array of msg/Joints messages.
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
ROSIDL_GENERATOR_C_PUBLIC_champ_msgs
bool
champ_msgs__msg__Joints__Sequence__copy(
  const champ_msgs__msg__Joints__Sequence * input,
  champ_msgs__msg__Joints__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CHAMP_MSGS__MSG__DETAIL__JOINTS__FUNCTIONS_H_
