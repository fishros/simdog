// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lio_sam:msg/CloudInfo.idl
// generated code does not contain a copyright notice
#include "lio_sam/msg/detail/cloud_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `start_ring_index`
// Member `end_ring_index`
// Member `point_col_ind`
// Member `point_range`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `cloud_deskewed`
// Member `cloud_corner`
// Member `cloud_surface`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"

bool
lio_sam__msg__CloudInfo__init(lio_sam__msg__CloudInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    lio_sam__msg__CloudInfo__fini(msg);
    return false;
  }
  // start_ring_index
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->start_ring_index, 0)) {
    lio_sam__msg__CloudInfo__fini(msg);
    return false;
  }
  // end_ring_index
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->end_ring_index, 0)) {
    lio_sam__msg__CloudInfo__fini(msg);
    return false;
  }
  // point_col_ind
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->point_col_ind, 0)) {
    lio_sam__msg__CloudInfo__fini(msg);
    return false;
  }
  // point_range
  if (!rosidl_runtime_c__float__Sequence__init(&msg->point_range, 0)) {
    lio_sam__msg__CloudInfo__fini(msg);
    return false;
  }
  // imu_available
  // odom_available
  // imu_roll_init
  // imu_pitch_init
  // imu_yaw_init
  // initial_guess_x
  // initial_guess_y
  // initial_guess_z
  // initial_guess_roll
  // initial_guess_pitch
  // initial_guess_yaw
  // cloud_deskewed
  if (!sensor_msgs__msg__PointCloud2__init(&msg->cloud_deskewed)) {
    lio_sam__msg__CloudInfo__fini(msg);
    return false;
  }
  // cloud_corner
  if (!sensor_msgs__msg__PointCloud2__init(&msg->cloud_corner)) {
    lio_sam__msg__CloudInfo__fini(msg);
    return false;
  }
  // cloud_surface
  if (!sensor_msgs__msg__PointCloud2__init(&msg->cloud_surface)) {
    lio_sam__msg__CloudInfo__fini(msg);
    return false;
  }
  return true;
}

void
lio_sam__msg__CloudInfo__fini(lio_sam__msg__CloudInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // start_ring_index
  rosidl_runtime_c__int32__Sequence__fini(&msg->start_ring_index);
  // end_ring_index
  rosidl_runtime_c__int32__Sequence__fini(&msg->end_ring_index);
  // point_col_ind
  rosidl_runtime_c__int32__Sequence__fini(&msg->point_col_ind);
  // point_range
  rosidl_runtime_c__float__Sequence__fini(&msg->point_range);
  // imu_available
  // odom_available
  // imu_roll_init
  // imu_pitch_init
  // imu_yaw_init
  // initial_guess_x
  // initial_guess_y
  // initial_guess_z
  // initial_guess_roll
  // initial_guess_pitch
  // initial_guess_yaw
  // cloud_deskewed
  sensor_msgs__msg__PointCloud2__fini(&msg->cloud_deskewed);
  // cloud_corner
  sensor_msgs__msg__PointCloud2__fini(&msg->cloud_corner);
  // cloud_surface
  sensor_msgs__msg__PointCloud2__fini(&msg->cloud_surface);
}

bool
lio_sam__msg__CloudInfo__are_equal(const lio_sam__msg__CloudInfo * lhs, const lio_sam__msg__CloudInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // start_ring_index
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->start_ring_index), &(rhs->start_ring_index)))
  {
    return false;
  }
  // end_ring_index
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->end_ring_index), &(rhs->end_ring_index)))
  {
    return false;
  }
  // point_col_ind
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->point_col_ind), &(rhs->point_col_ind)))
  {
    return false;
  }
  // point_range
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->point_range), &(rhs->point_range)))
  {
    return false;
  }
  // imu_available
  if (lhs->imu_available != rhs->imu_available) {
    return false;
  }
  // odom_available
  if (lhs->odom_available != rhs->odom_available) {
    return false;
  }
  // imu_roll_init
  if (lhs->imu_roll_init != rhs->imu_roll_init) {
    return false;
  }
  // imu_pitch_init
  if (lhs->imu_pitch_init != rhs->imu_pitch_init) {
    return false;
  }
  // imu_yaw_init
  if (lhs->imu_yaw_init != rhs->imu_yaw_init) {
    return false;
  }
  // initial_guess_x
  if (lhs->initial_guess_x != rhs->initial_guess_x) {
    return false;
  }
  // initial_guess_y
  if (lhs->initial_guess_y != rhs->initial_guess_y) {
    return false;
  }
  // initial_guess_z
  if (lhs->initial_guess_z != rhs->initial_guess_z) {
    return false;
  }
  // initial_guess_roll
  if (lhs->initial_guess_roll != rhs->initial_guess_roll) {
    return false;
  }
  // initial_guess_pitch
  if (lhs->initial_guess_pitch != rhs->initial_guess_pitch) {
    return false;
  }
  // initial_guess_yaw
  if (lhs->initial_guess_yaw != rhs->initial_guess_yaw) {
    return false;
  }
  // cloud_deskewed
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->cloud_deskewed), &(rhs->cloud_deskewed)))
  {
    return false;
  }
  // cloud_corner
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->cloud_corner), &(rhs->cloud_corner)))
  {
    return false;
  }
  // cloud_surface
  if (!sensor_msgs__msg__PointCloud2__are_equal(
      &(lhs->cloud_surface), &(rhs->cloud_surface)))
  {
    return false;
  }
  return true;
}

bool
lio_sam__msg__CloudInfo__copy(
  const lio_sam__msg__CloudInfo * input,
  lio_sam__msg__CloudInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // start_ring_index
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->start_ring_index), &(output->start_ring_index)))
  {
    return false;
  }
  // end_ring_index
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->end_ring_index), &(output->end_ring_index)))
  {
    return false;
  }
  // point_col_ind
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->point_col_ind), &(output->point_col_ind)))
  {
    return false;
  }
  // point_range
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->point_range), &(output->point_range)))
  {
    return false;
  }
  // imu_available
  output->imu_available = input->imu_available;
  // odom_available
  output->odom_available = input->odom_available;
  // imu_roll_init
  output->imu_roll_init = input->imu_roll_init;
  // imu_pitch_init
  output->imu_pitch_init = input->imu_pitch_init;
  // imu_yaw_init
  output->imu_yaw_init = input->imu_yaw_init;
  // initial_guess_x
  output->initial_guess_x = input->initial_guess_x;
  // initial_guess_y
  output->initial_guess_y = input->initial_guess_y;
  // initial_guess_z
  output->initial_guess_z = input->initial_guess_z;
  // initial_guess_roll
  output->initial_guess_roll = input->initial_guess_roll;
  // initial_guess_pitch
  output->initial_guess_pitch = input->initial_guess_pitch;
  // initial_guess_yaw
  output->initial_guess_yaw = input->initial_guess_yaw;
  // cloud_deskewed
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->cloud_deskewed), &(output->cloud_deskewed)))
  {
    return false;
  }
  // cloud_corner
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->cloud_corner), &(output->cloud_corner)))
  {
    return false;
  }
  // cloud_surface
  if (!sensor_msgs__msg__PointCloud2__copy(
      &(input->cloud_surface), &(output->cloud_surface)))
  {
    return false;
  }
  return true;
}

lio_sam__msg__CloudInfo *
lio_sam__msg__CloudInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lio_sam__msg__CloudInfo * msg = (lio_sam__msg__CloudInfo *)allocator.allocate(sizeof(lio_sam__msg__CloudInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lio_sam__msg__CloudInfo));
  bool success = lio_sam__msg__CloudInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lio_sam__msg__CloudInfo__destroy(lio_sam__msg__CloudInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lio_sam__msg__CloudInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lio_sam__msg__CloudInfo__Sequence__init(lio_sam__msg__CloudInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lio_sam__msg__CloudInfo * data = NULL;

  if (size) {
    data = (lio_sam__msg__CloudInfo *)allocator.zero_allocate(size, sizeof(lio_sam__msg__CloudInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lio_sam__msg__CloudInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lio_sam__msg__CloudInfo__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
lio_sam__msg__CloudInfo__Sequence__fini(lio_sam__msg__CloudInfo__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      lio_sam__msg__CloudInfo__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

lio_sam__msg__CloudInfo__Sequence *
lio_sam__msg__CloudInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lio_sam__msg__CloudInfo__Sequence * array = (lio_sam__msg__CloudInfo__Sequence *)allocator.allocate(sizeof(lio_sam__msg__CloudInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lio_sam__msg__CloudInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lio_sam__msg__CloudInfo__Sequence__destroy(lio_sam__msg__CloudInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lio_sam__msg__CloudInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lio_sam__msg__CloudInfo__Sequence__are_equal(const lio_sam__msg__CloudInfo__Sequence * lhs, const lio_sam__msg__CloudInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lio_sam__msg__CloudInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lio_sam__msg__CloudInfo__Sequence__copy(
  const lio_sam__msg__CloudInfo__Sequence * input,
  lio_sam__msg__CloudInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lio_sam__msg__CloudInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    lio_sam__msg__CloudInfo * data =
      (lio_sam__msg__CloudInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lio_sam__msg__CloudInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          lio_sam__msg__CloudInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!lio_sam__msg__CloudInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
