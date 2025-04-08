// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from champ_msgs:msg/Velocities.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__VELOCITIES__STRUCT_H_
#define CHAMP_MSGS__MSG__DETAIL__VELOCITIES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Velocities in the package champ_msgs.
typedef struct champ_msgs__msg__Velocities
{
  float linear_x;
  float linear_y;
  float angular_z;
} champ_msgs__msg__Velocities;

// Struct for a sequence of champ_msgs__msg__Velocities.
typedef struct champ_msgs__msg__Velocities__Sequence
{
  champ_msgs__msg__Velocities * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} champ_msgs__msg__Velocities__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHAMP_MSGS__MSG__DETAIL__VELOCITIES__STRUCT_H_
