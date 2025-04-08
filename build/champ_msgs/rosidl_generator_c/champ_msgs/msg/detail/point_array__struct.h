// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from champ_msgs:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__POINT_ARRAY__STRUCT_H_
#define CHAMP_MSGS__MSG__DETAIL__POINT_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'lf'
// Member 'rf'
// Member 'lh'
// Member 'rh'
#include "champ_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/PointArray in the package champ_msgs.
typedef struct champ_msgs__msg__PointArray
{
  champ_msgs__msg__Point lf;
  champ_msgs__msg__Point rf;
  champ_msgs__msg__Point lh;
  champ_msgs__msg__Point rh;
} champ_msgs__msg__PointArray;

// Struct for a sequence of champ_msgs__msg__PointArray.
typedef struct champ_msgs__msg__PointArray__Sequence
{
  champ_msgs__msg__PointArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} champ_msgs__msg__PointArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHAMP_MSGS__MSG__DETAIL__POINT_ARRAY__STRUCT_H_
