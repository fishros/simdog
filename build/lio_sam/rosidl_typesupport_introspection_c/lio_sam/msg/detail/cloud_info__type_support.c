// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lio_sam:msg/CloudInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lio_sam/msg/detail/cloud_info__rosidl_typesupport_introspection_c.h"
#include "lio_sam/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lio_sam/msg/detail/cloud_info__functions.h"
#include "lio_sam/msg/detail/cloud_info__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `start_ring_index`
// Member `end_ring_index`
// Member `point_col_ind`
// Member `point_range`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `cloud_deskewed`
// Member `cloud_corner`
// Member `cloud_surface`
#include "sensor_msgs/msg/point_cloud2.h"
// Member `cloud_deskewed`
// Member `cloud_corner`
// Member `cloud_surface`
#include "sensor_msgs/msg/detail/point_cloud2__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lio_sam__msg__CloudInfo__init(message_memory);
}

void lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_fini_function(void * message_memory)
{
  lio_sam__msg__CloudInfo__fini(message_memory);
}

size_t lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__size_function__CloudInfo__start_ring_index(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_const_function__CloudInfo__start_ring_index(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_function__CloudInfo__start_ring_index(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__fetch_function__CloudInfo__start_ring_index(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_const_function__CloudInfo__start_ring_index(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__assign_function__CloudInfo__start_ring_index(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_function__CloudInfo__start_ring_index(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__resize_function__CloudInfo__start_ring_index(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__size_function__CloudInfo__end_ring_index(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_const_function__CloudInfo__end_ring_index(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_function__CloudInfo__end_ring_index(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__fetch_function__CloudInfo__end_ring_index(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_const_function__CloudInfo__end_ring_index(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__assign_function__CloudInfo__end_ring_index(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_function__CloudInfo__end_ring_index(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__resize_function__CloudInfo__end_ring_index(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__size_function__CloudInfo__point_col_ind(
  const void * untyped_member)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return member->size;
}

const void * lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_const_function__CloudInfo__point_col_ind(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int32__Sequence * member =
    (const rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_function__CloudInfo__point_col_ind(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  return &member->data[index];
}

void lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__fetch_function__CloudInfo__point_col_ind(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int32_t * item =
    ((const int32_t *)
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_const_function__CloudInfo__point_col_ind(untyped_member, index));
  int32_t * value =
    (int32_t *)(untyped_value);
  *value = *item;
}

void lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__assign_function__CloudInfo__point_col_ind(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int32_t * item =
    ((int32_t *)
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_function__CloudInfo__point_col_ind(untyped_member, index));
  const int32_t * value =
    (const int32_t *)(untyped_value);
  *item = *value;
}

bool lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__resize_function__CloudInfo__point_col_ind(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int32__Sequence * member =
    (rosidl_runtime_c__int32__Sequence *)(untyped_member);
  rosidl_runtime_c__int32__Sequence__fini(member);
  return rosidl_runtime_c__int32__Sequence__init(member, size);
}

size_t lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__size_function__CloudInfo__point_range(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_const_function__CloudInfo__point_range(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_function__CloudInfo__point_range(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__fetch_function__CloudInfo__point_range(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_const_function__CloudInfo__point_range(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__assign_function__CloudInfo__point_range(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_function__CloudInfo__point_range(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__resize_function__CloudInfo__point_range(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_member_array[19] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "start_ring_index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, start_ring_index),  // bytes offset in struct
    NULL,  // default value
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__size_function__CloudInfo__start_ring_index,  // size() function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_const_function__CloudInfo__start_ring_index,  // get_const(index) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_function__CloudInfo__start_ring_index,  // get(index) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__fetch_function__CloudInfo__start_ring_index,  // fetch(index, &value) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__assign_function__CloudInfo__start_ring_index,  // assign(index, value) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__resize_function__CloudInfo__start_ring_index  // resize(index) function pointer
  },
  {
    "end_ring_index",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, end_ring_index),  // bytes offset in struct
    NULL,  // default value
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__size_function__CloudInfo__end_ring_index,  // size() function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_const_function__CloudInfo__end_ring_index,  // get_const(index) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_function__CloudInfo__end_ring_index,  // get(index) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__fetch_function__CloudInfo__end_ring_index,  // fetch(index, &value) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__assign_function__CloudInfo__end_ring_index,  // assign(index, value) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__resize_function__CloudInfo__end_ring_index  // resize(index) function pointer
  },
  {
    "point_col_ind",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, point_col_ind),  // bytes offset in struct
    NULL,  // default value
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__size_function__CloudInfo__point_col_ind,  // size() function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_const_function__CloudInfo__point_col_ind,  // get_const(index) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_function__CloudInfo__point_col_ind,  // get(index) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__fetch_function__CloudInfo__point_col_ind,  // fetch(index, &value) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__assign_function__CloudInfo__point_col_ind,  // assign(index, value) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__resize_function__CloudInfo__point_col_ind  // resize(index) function pointer
  },
  {
    "point_range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, point_range),  // bytes offset in struct
    NULL,  // default value
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__size_function__CloudInfo__point_range,  // size() function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_const_function__CloudInfo__point_range,  // get_const(index) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__get_function__CloudInfo__point_range,  // get(index) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__fetch_function__CloudInfo__point_range,  // fetch(index, &value) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__assign_function__CloudInfo__point_range,  // assign(index, value) function pointer
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__resize_function__CloudInfo__point_range  // resize(index) function pointer
  },
  {
    "imu_available",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, imu_available),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "odom_available",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, odom_available),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imu_roll_init",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, imu_roll_init),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imu_pitch_init",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, imu_pitch_init),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "imu_yaw_init",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, imu_yaw_init),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "initial_guess_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, initial_guess_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "initial_guess_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, initial_guess_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "initial_guess_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, initial_guess_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "initial_guess_roll",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, initial_guess_roll),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "initial_guess_pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, initial_guess_pitch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "initial_guess_yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, initial_guess_yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cloud_deskewed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, cloud_deskewed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cloud_corner",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, cloud_corner),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cloud_surface",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lio_sam__msg__CloudInfo, cloud_surface),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_members = {
  "lio_sam__msg",  // message namespace
  "CloudInfo",  // message name
  19,  // number of fields
  sizeof(lio_sam__msg__CloudInfo),
  lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_member_array,  // message members
  lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_type_support_handle = {
  0,
  &lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lio_sam
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lio_sam, msg, CloudInfo)() {
  lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_member_array[16].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_member_array[17].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_member_array[18].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  if (!lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_type_support_handle.typesupport_identifier) {
    lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &lio_sam__msg__CloudInfo__rosidl_typesupport_introspection_c__CloudInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
