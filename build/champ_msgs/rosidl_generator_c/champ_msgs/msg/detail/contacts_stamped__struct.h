// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from champ_msgs:msg/ContactsStamped.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__CONTACTS_STAMPED__STRUCT_H_
#define CHAMP_MSGS__MSG__DETAIL__CONTACTS_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'contacts'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/ContactsStamped in the package champ_msgs.
typedef struct champ_msgs__msg__ContactsStamped
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__boolean__Sequence contacts;
} champ_msgs__msg__ContactsStamped;

// Struct for a sequence of champ_msgs__msg__ContactsStamped.
typedef struct champ_msgs__msg__ContactsStamped__Sequence
{
  champ_msgs__msg__ContactsStamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} champ_msgs__msg__ContactsStamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CHAMP_MSGS__MSG__DETAIL__CONTACTS_STAMPED__STRUCT_H_
