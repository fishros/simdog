// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from champ_msgs:msg/PID.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__PID__STRUCT_H_
#define CHAMP_MSGS__MSG__DETAIL__PID__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PID in the package champ_msgs.
typedef struct champ_msgs__msg__PID
{
  float p;
  float d;
  float i;
} champ_msgs__msg__PID;

// Struct for a sequence of champ_msgs__msg__PID.
typedef struct champ_msgs__msg__PID__Sequence
{
  champ_msgs__msg__PID * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} champ_msgs__msg__PID__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHAMP_MSGS__MSG__DETAIL__PID__STRUCT_H_
