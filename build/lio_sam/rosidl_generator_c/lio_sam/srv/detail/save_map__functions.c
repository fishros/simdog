// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lio_sam:srv/SaveMap.idl
// generated code does not contain a copyright notice
#include "lio_sam/srv/detail/save_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `destination`
#include "rosidl_runtime_c/string_functions.h"

bool
lio_sam__srv__SaveMap_Request__init(lio_sam__srv__SaveMap_Request * msg)
{
  if (!msg) {
    return false;
  }
  // resolution
  // destination
  if (!rosidl_runtime_c__String__init(&msg->destination)) {
    lio_sam__srv__SaveMap_Request__fini(msg);
    return false;
  }
  return true;
}

void
lio_sam__srv__SaveMap_Request__fini(lio_sam__srv__SaveMap_Request * msg)
{
  if (!msg) {
    return;
  }
  // resolution
  // destination
  rosidl_runtime_c__String__fini(&msg->destination);
}

bool
lio_sam__srv__SaveMap_Request__are_equal(const lio_sam__srv__SaveMap_Request * lhs, const lio_sam__srv__SaveMap_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // resolution
  if (lhs->resolution != rhs->resolution) {
    return false;
  }
  // destination
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->destination), &(rhs->destination)))
  {
    return false;
  }
  return true;
}

bool
lio_sam__srv__SaveMap_Request__copy(
  const lio_sam__srv__SaveMap_Request * input,
  lio_sam__srv__SaveMap_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // resolution
  output->resolution = input->resolution;
  // destination
  if (!rosidl_runtime_c__String__copy(
      &(input->destination), &(output->destination)))
  {
    return false;
  }
  return true;
}

lio_sam__srv__SaveMap_Request *
lio_sam__srv__SaveMap_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lio_sam__srv__SaveMap_Request * msg = (lio_sam__srv__SaveMap_Request *)allocator.allocate(sizeof(lio_sam__srv__SaveMap_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lio_sam__srv__SaveMap_Request));
  bool success = lio_sam__srv__SaveMap_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lio_sam__srv__SaveMap_Request__destroy(lio_sam__srv__SaveMap_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lio_sam__srv__SaveMap_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lio_sam__srv__SaveMap_Request__Sequence__init(lio_sam__srv__SaveMap_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lio_sam__srv__SaveMap_Request * data = NULL;

  if (size) {
    data = (lio_sam__srv__SaveMap_Request *)allocator.zero_allocate(size, sizeof(lio_sam__srv__SaveMap_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lio_sam__srv__SaveMap_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lio_sam__srv__SaveMap_Request__fini(&data[i - 1]);
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
lio_sam__srv__SaveMap_Request__Sequence__fini(lio_sam__srv__SaveMap_Request__Sequence * array)
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
      lio_sam__srv__SaveMap_Request__fini(&array->data[i]);
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

lio_sam__srv__SaveMap_Request__Sequence *
lio_sam__srv__SaveMap_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lio_sam__srv__SaveMap_Request__Sequence * array = (lio_sam__srv__SaveMap_Request__Sequence *)allocator.allocate(sizeof(lio_sam__srv__SaveMap_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lio_sam__srv__SaveMap_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lio_sam__srv__SaveMap_Request__Sequence__destroy(lio_sam__srv__SaveMap_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lio_sam__srv__SaveMap_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lio_sam__srv__SaveMap_Request__Sequence__are_equal(const lio_sam__srv__SaveMap_Request__Sequence * lhs, const lio_sam__srv__SaveMap_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lio_sam__srv__SaveMap_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lio_sam__srv__SaveMap_Request__Sequence__copy(
  const lio_sam__srv__SaveMap_Request__Sequence * input,
  lio_sam__srv__SaveMap_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lio_sam__srv__SaveMap_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    lio_sam__srv__SaveMap_Request * data =
      (lio_sam__srv__SaveMap_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lio_sam__srv__SaveMap_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          lio_sam__srv__SaveMap_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!lio_sam__srv__SaveMap_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
lio_sam__srv__SaveMap_Response__init(lio_sam__srv__SaveMap_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
lio_sam__srv__SaveMap_Response__fini(lio_sam__srv__SaveMap_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
lio_sam__srv__SaveMap_Response__are_equal(const lio_sam__srv__SaveMap_Response * lhs, const lio_sam__srv__SaveMap_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
lio_sam__srv__SaveMap_Response__copy(
  const lio_sam__srv__SaveMap_Response * input,
  lio_sam__srv__SaveMap_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

lio_sam__srv__SaveMap_Response *
lio_sam__srv__SaveMap_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lio_sam__srv__SaveMap_Response * msg = (lio_sam__srv__SaveMap_Response *)allocator.allocate(sizeof(lio_sam__srv__SaveMap_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lio_sam__srv__SaveMap_Response));
  bool success = lio_sam__srv__SaveMap_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
lio_sam__srv__SaveMap_Response__destroy(lio_sam__srv__SaveMap_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    lio_sam__srv__SaveMap_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
lio_sam__srv__SaveMap_Response__Sequence__init(lio_sam__srv__SaveMap_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lio_sam__srv__SaveMap_Response * data = NULL;

  if (size) {
    data = (lio_sam__srv__SaveMap_Response *)allocator.zero_allocate(size, sizeof(lio_sam__srv__SaveMap_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lio_sam__srv__SaveMap_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lio_sam__srv__SaveMap_Response__fini(&data[i - 1]);
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
lio_sam__srv__SaveMap_Response__Sequence__fini(lio_sam__srv__SaveMap_Response__Sequence * array)
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
      lio_sam__srv__SaveMap_Response__fini(&array->data[i]);
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

lio_sam__srv__SaveMap_Response__Sequence *
lio_sam__srv__SaveMap_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  lio_sam__srv__SaveMap_Response__Sequence * array = (lio_sam__srv__SaveMap_Response__Sequence *)allocator.allocate(sizeof(lio_sam__srv__SaveMap_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = lio_sam__srv__SaveMap_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
lio_sam__srv__SaveMap_Response__Sequence__destroy(lio_sam__srv__SaveMap_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    lio_sam__srv__SaveMap_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
lio_sam__srv__SaveMap_Response__Sequence__are_equal(const lio_sam__srv__SaveMap_Response__Sequence * lhs, const lio_sam__srv__SaveMap_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!lio_sam__srv__SaveMap_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
lio_sam__srv__SaveMap_Response__Sequence__copy(
  const lio_sam__srv__SaveMap_Response__Sequence * input,
  lio_sam__srv__SaveMap_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(lio_sam__srv__SaveMap_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    lio_sam__srv__SaveMap_Response * data =
      (lio_sam__srv__SaveMap_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!lio_sam__srv__SaveMap_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          lio_sam__srv__SaveMap_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!lio_sam__srv__SaveMap_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
