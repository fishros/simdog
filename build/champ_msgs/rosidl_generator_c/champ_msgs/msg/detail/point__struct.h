// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from champ_msgs:msg/Point.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__POINT__STRUCT_H_
#define CHAMP_MSGS__MSG__DETAIL__POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Point in the package champ_msgs.
typedef struct champ_msgs__msg__Point
{
  float x;
  float y;
  float z;
} champ_msgs__msg__Point;

// Struct for a sequence of champ_msgs__msg__Point.
typedef struct champ_msgs__msg__Point__Sequence
{
  champ_msgs__msg__Point * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} champ_msgs__msg__Point__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHAMP_MSGS__MSG__DETAIL__POINT__STRUCT_H_
