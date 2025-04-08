// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lio_sam:srv/SaveMap.idl
// generated code does not contain a copyright notice

#ifndef LIO_SAM__SRV__DETAIL__SAVE_MAP__STRUCT_H_
#define LIO_SAM__SRV__DETAIL__SAVE_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'destination'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/SaveMap in the package lio_sam.
typedef struct lio_sam__srv__SaveMap_Request
{
  float resolution;
  rosidl_runtime_c__String destination;
} lio_sam__srv__SaveMap_Request;

// Struct for a sequence of lio_sam__srv__SaveMap_Request.
typedef struct lio_sam__srv__SaveMap_Request__Sequence
{
  lio_sam__srv__SaveMap_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lio_sam__srv__SaveMap_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SaveMap in the package lio_sam.
typedef struct lio_sam__srv__SaveMap_Response
{
  bool success;
} lio_sam__srv__SaveMap_Response;

// Struct for a sequence of lio_sam__srv__SaveMap_Response.
typedef struct lio_sam__srv__SaveMap_Response__Sequence
{
  lio_sam__srv__SaveMap_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lio_sam__srv__SaveMap_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIO_SAM__SRV__DETAIL__SAVE_MAP__STRUCT_H_
