// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from champ_msgs:msg/PID.idl
// generated code does not contain a copyright notice
#include "champ_msgs/msg/detail/pid__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
champ_msgs__msg__PID__init(champ_msgs__msg__PID * msg)
{
  if (!msg) {
    return false;
  }
  // p
  // d
  // i
  return true;
}

void
champ_msgs__msg__PID__fini(champ_msgs__msg__PID * msg)
{
  if (!msg) {
    return;
  }
  // p
  // d
  // i
}

bool
champ_msgs__msg__PID__are_equal(const champ_msgs__msg__PID * lhs, const champ_msgs__msg__PID * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // p
  if (lhs->p != rhs->p) {
    return false;
  }
  // d
  if (lhs->d != rhs->d) {
    return false;
  }
  // i
  if (lhs->i != rhs->i) {
    return false;
  }
  return true;
}

bool
champ_msgs__msg__PID__copy(
  const champ_msgs__msg__PID * input,
  champ_msgs__msg__PID * output)
{
  if (!input || !output) {
    return false;
  }
  // p
  output->p = input->p;
  // d
  output->d = input->d;
  // i
  output->i = input->i;
  return true;
}

champ_msgs__msg__PID *
champ_msgs__msg__PID__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  champ_msgs__msg__PID * msg = (champ_msgs__msg__PID *)allocator.allocate(sizeof(champ_msgs__msg__PID), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(champ_msgs__msg__PID));
  bool success = champ_msgs__msg__PID__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
champ_msgs__msg__PID__destroy(champ_msgs__msg__PID * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    champ_msgs__msg__PID__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
champ_msgs__msg__PID__Sequence__init(champ_msgs__msg__PID__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  champ_msgs__msg__PID * data = NULL;

  if (size) {
    data = (champ_msgs__msg__PID *)allocator.zero_allocate(size, sizeof(champ_msgs__msg__PID), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = champ_msgs__msg__PID__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        champ_msgs__msg__PID__fini(&data[i - 1]);
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
champ_msgs__msg__PID__Sequence__fini(champ_msgs__msg__PID__Sequence * array)
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
      champ_msgs__msg__PID__fini(&array->data[i]);
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

champ_msgs__msg__PID__Sequence *
champ_msgs__msg__PID__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  champ_msgs__msg__PID__Sequence * array = (champ_msgs__msg__PID__Sequence *)allocator.allocate(sizeof(champ_msgs__msg__PID__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = champ_msgs__msg__PID__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
champ_msgs__msg__PID__Sequence__destroy(champ_msgs__msg__PID__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    champ_msgs__msg__PID__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
champ_msgs__msg__PID__Sequence__are_equal(const champ_msgs__msg__PID__Sequence * lhs, const champ_msgs__msg__PID__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!champ_msgs__msg__PID__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
champ_msgs__msg__PID__Sequence__copy(
  const champ_msgs__msg__PID__Sequence * input,
  champ_msgs__msg__PID__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(champ_msgs__msg__PID);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    champ_msgs__msg__PID * data =
      (champ_msgs__msg__PID *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!champ_msgs__msg__PID__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          champ_msgs__msg__PID__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!champ_msgs__msg__PID__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
