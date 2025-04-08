// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from champ_msgs:msg/Joints.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "champ_msgs/msg/detail/joints__rosidl_typesupport_introspection_c.h"
#include "champ_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "champ_msgs/msg/detail/joints__functions.h"
#include "champ_msgs/msg/detail/joints__struct.h"


// Include directives for member types
// Member `position`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__Joints_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  champ_msgs__msg__Joints__init(message_memory);
}

void champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__Joints_fini_function(void * message_memory)
{
  champ_msgs__msg__Joints__fini(message_memory);
}

size_t champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__size_function__Joints__position(
  const void * untyped_member)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return member->size;
}

const void * champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__get_const_function__Joints__position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__float__Sequence * member =
    (const rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void * champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__get_function__Joints__position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  return &member->data[index];
}

void champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__fetch_function__Joints__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const float * item =
    ((const float *)
    champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__get_const_function__Joints__position(untyped_member, index));
  float * value =
    (float *)(untyped_value);
  *value = *item;
}

void champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__assign_function__Joints__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  float * item =
    ((float *)
    champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__get_function__Joints__position(untyped_member, index));
  const float * value =
    (const float *)(untyped_value);
  *item = *value;
}

bool champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__resize_function__Joints__position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__float__Sequence * member =
    (rosidl_runtime_c__float__Sequence *)(untyped_member);
  rosidl_runtime_c__float__Sequence__fini(member);
  return rosidl_runtime_c__float__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__Joints_message_member_array[1] = {
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(champ_msgs__msg__Joints, position),  // bytes offset in struct
    NULL,  // default value
    champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__size_function__Joints__position,  // size() function pointer
    champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__get_const_function__Joints__position,  // get_const(index) function pointer
    champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__get_function__Joints__position,  // get(index) function pointer
    champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__fetch_function__Joints__position,  // fetch(index, &value) function pointer
    champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__assign_function__Joints__position,  // assign(index, value) function pointer
    champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__resize_function__Joints__position  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__Joints_message_members = {
  "champ_msgs__msg",  // message namespace
  "Joints",  // message name
  1,  // number of fields
  sizeof(champ_msgs__msg__Joints),
  champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__Joints_message_member_array,  // message members
  champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__Joints_init_function,  // function to initialize message memory (memory has to be allocated)
  champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__Joints_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__Joints_message_type_support_handle = {
  0,
  &champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__Joints_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_champ_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, champ_msgs, msg, Joints)() {
  if (!champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__Joints_message_type_support_handle.typesupport_identifier) {
    champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__Joints_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &champ_msgs__msg__Joints__rosidl_typesupport_introspection_c__Joints_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
