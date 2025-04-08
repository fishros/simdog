// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from champ_msgs:msg/Joints.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__JOINTS__STRUCT_H_
#define CHAMP_MSGS__MSG__DETAIL__JOINTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'position'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Joints in the package champ_msgs.
typedef struct champ_msgs__msg__Joints
{
  rosidl_runtime_c__float__Sequence position;
} champ_msgs__msg__Joints;

// Struct for a sequence of champ_msgs__msg__Joints.
typedef struct champ_msgs__msg__Joints__Sequence
{
  champ_msgs__msg__Joints * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} champ_msgs__msg__Joints__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHAMP_MSGS__MSG__DETAIL__JOINTS__STRUCT_H_
