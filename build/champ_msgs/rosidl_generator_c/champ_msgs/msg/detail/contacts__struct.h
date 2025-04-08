// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from champ_msgs:msg/Contacts.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__CONTACTS__STRUCT_H_
#define CHAMP_MSGS__MSG__DETAIL__CONTACTS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'contacts'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Contacts in the package champ_msgs.
typedef struct champ_msgs__msg__Contacts
{
  rosidl_runtime_c__boolean__Sequence contacts;
} champ_msgs__msg__Contacts;

// Struct for a sequence of champ_msgs__msg__Contacts.
typedef struct champ_msgs__msg__Contacts__Sequence
{
  champ_msgs__msg__Contacts * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} champ_msgs__msg__Contacts__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHAMP_MSGS__MSG__DETAIL__CONTACTS__STRUCT_H_
