// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from champ_msgs:msg/Imu.idl
// generated code does not contain a copyright notice
#include "champ_msgs/msg/detail/imu__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `orientation`
#include "geometry_msgs/msg/detail/quaternion__functions.h"
// Member `linear_acceleration`
// Member `angular_velocity`
// Member `magnetic_field`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
champ_msgs__msg__Imu__init(champ_msgs__msg__Imu * msg)
{
  if (!msg) {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__init(&msg->orientation)) {
    champ_msgs__msg__Imu__fini(msg);
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__init(&msg->linear_acceleration)) {
    champ_msgs__msg__Imu__fini(msg);
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->angular_velocity)) {
    champ_msgs__msg__Imu__fini(msg);
    return false;
  }
  // magnetic_field
  if (!geometry_msgs__msg__Vector3__init(&msg->magnetic_field)) {
    champ_msgs__msg__Imu__fini(msg);
    return false;
  }
  return true;
}

void
champ_msgs__msg__Imu__fini(champ_msgs__msg__Imu * msg)
{
  if (!msg) {
    return;
  }
  // orientation
  geometry_msgs__msg__Quaternion__fini(&msg->orientation);
  // linear_acceleration
  geometry_msgs__msg__Vector3__fini(&msg->linear_acceleration);
  // angular_velocity
  geometry_msgs__msg__Vector3__fini(&msg->angular_velocity);
  // magnetic_field
  geometry_msgs__msg__Vector3__fini(&msg->magnetic_field);
}

bool
champ_msgs__msg__Imu__are_equal(const champ_msgs__msg__Imu * lhs, const champ_msgs__msg__Imu * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__are_equal(
      &(lhs->orientation), &(rhs->orientation)))
  {
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->linear_acceleration), &(rhs->linear_acceleration)))
  {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->angular_velocity), &(rhs->angular_velocity)))
  {
    return false;
  }
  // magnetic_field
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->magnetic_field), &(rhs->magnetic_field)))
  {
    return false;
  }
  return true;
}

bool
champ_msgs__msg__Imu__copy(
  const champ_msgs__msg__Imu * input,
  champ_msgs__msg__Imu * output)
{
  if (!input || !output) {
    return false;
  }
  // orientation
  if (!geometry_msgs__msg__Quaternion__copy(
      &(input->orientation), &(output->orientation)))
  {
    return false;
  }
  // linear_acceleration
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->linear_acceleration), &(output->linear_acceleration)))
  {
    return false;
  }
  // angular_velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->angular_velocity), &(output->angular_velocity)))
  {
    return false;
  }
  // magnetic_field
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->magnetic_field), &(output->magnetic_field)))
  {
    return false;
  }
  return true;
}

champ_msgs__msg__Imu *
champ_msgs__msg__Imu__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  champ_msgs__msg__Imu * msg = (champ_msgs__msg__Imu *)allocator.allocate(sizeof(champ_msgs__msg__Imu), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(champ_msgs__msg__Imu));
  bool success = champ_msgs__msg__Imu__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
champ_msgs__msg__Imu__destroy(champ_msgs__msg__Imu * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    champ_msgs__msg__Imu__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
champ_msgs__msg__Imu__Sequence__init(champ_msgs__msg__Imu__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  champ_msgs__msg__Imu * data = NULL;

  if (size) {
    data = (champ_msgs__msg__Imu *)allocator.zero_allocate(size, sizeof(champ_msgs__msg__Imu), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = champ_msgs__msg__Imu__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        champ_msgs__msg__Imu__fini(&data[i - 1]);
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
champ_msgs__msg__Imu__Sequence__fini(champ_msgs__msg__Imu__Sequence * array)
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
      champ_msgs__msg__Imu__fini(&array->data[i]);
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

champ_msgs__msg__Imu__Sequence *
champ_msgs__msg__Imu__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  champ_msgs__msg__Imu__Sequence * array = (champ_msgs__msg__Imu__Sequence *)allocator.allocate(sizeof(champ_msgs__msg__Imu__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = champ_msgs__msg__Imu__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
champ_msgs__msg__Imu__Sequence__destroy(champ_msgs__msg__Imu__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    champ_msgs__msg__Imu__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
champ_msgs__msg__Imu__Sequence__are_equal(const champ_msgs__msg__Imu__Sequence * lhs, const champ_msgs__msg__Imu__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!champ_msgs__msg__Imu__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
champ_msgs__msg__Imu__Sequence__copy(
  const champ_msgs__msg__Imu__Sequence * input,
  champ_msgs__msg__Imu__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(champ_msgs__msg__Imu);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    champ_msgs__msg__Imu * data =
      (champ_msgs__msg__Imu *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!champ_msgs__msg__Imu__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          champ_msgs__msg__Imu__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!champ_msgs__msg__Imu__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
