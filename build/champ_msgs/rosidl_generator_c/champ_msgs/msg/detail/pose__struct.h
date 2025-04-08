// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from champ_msgs:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__POSE__STRUCT_H_
#define CHAMP_MSGS__MSG__DETAIL__POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Pose in the package champ_msgs.
typedef struct champ_msgs__msg__Pose
{
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
} champ_msgs__msg__Pose;

// Struct for a sequence of champ_msgs__msg__Pose.
typedef struct champ_msgs__msg__Pose__Sequence
{
  champ_msgs__msg__Pose * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} champ_msgs__msg__Pose__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHAMP_MSGS__MSG__DETAIL__POSE__STRUCT_H_
