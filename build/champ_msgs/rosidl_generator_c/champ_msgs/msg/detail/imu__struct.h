// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from champ_msgs:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__IMU__STRUCT_H_
#define CHAMP_MSGS__MSG__DETAIL__IMU__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'linear_acceleration'
// Member 'angular_velocity'
// Member 'magnetic_field'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Imu in the package champ_msgs.
typedef struct champ_msgs__msg__Imu
{
  geometry_msgs__msg__Quaternion orientation;
  geometry_msgs__msg__Vector3 linear_acceleration;
  geometry_msgs__msg__Vector3 angular_velocity;
  geometry_msgs__msg__Vector3 magnetic_field;
} champ_msgs__msg__Imu;

// Struct for a sequence of champ_msgs__msg__Imu.
typedef struct champ_msgs__msg__Imu__Sequence
{
  champ_msgs__msg__Imu * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} champ_msgs__msg__Imu__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHAMP_MSGS__MSG__DETAIL__IMU__STRUCT_H_
