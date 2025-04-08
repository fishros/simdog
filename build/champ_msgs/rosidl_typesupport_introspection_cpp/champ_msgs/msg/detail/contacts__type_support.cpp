// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from champ_msgs:msg/Contacts.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "champ_msgs/msg/detail/contacts__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace champ_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Contacts_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) champ_msgs::msg::Contacts(_init);
}

void Contacts_fini_function(void * message_memory)
{
  auto typed_message = static_cast<champ_msgs::msg::Contacts *>(message_memory);
  typed_message->~Contacts();
}

size_t size_function__Contacts__contacts(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<bool> *>(untyped_member);
  return member->size();
}

void fetch_function__Contacts__contacts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & member = *reinterpret_cast<const std::vector<bool> *>(untyped_member);
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = member[index];
}

void assign_function__Contacts__contacts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & member = *reinterpret_cast<std::vector<bool> *>(untyped_member);
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  member[index] = value;
}

void resize_function__Contacts__contacts(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<bool> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Contacts_message_member_array[1] = {
  {
    "contacts",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(champ_msgs::msg::Contacts, contacts),  // bytes offset in struct
    nullptr,  // default value
    size_function__Contacts__contacts,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    fetch_function__Contacts__contacts,  // fetch(index, &value) function pointer
    assign_function__Contacts__contacts,  // assign(index, value) function pointer
    resize_function__Contacts__contacts  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Contacts_message_members = {
  "champ_msgs::msg",  // message namespace
  "Contacts",  // message name
  1,  // number of fields
  sizeof(champ_msgs::msg::Contacts),
  Contacts_message_member_array,  // message members
  Contacts_init_function,  // function to initialize message memory (memory has to be allocated)
  Contacts_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Contacts_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Contacts_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace champ_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<champ_msgs::msg::Contacts>()
{
  return &::champ_msgs::msg::rosidl_typesupport_introspection_cpp::Contacts_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, champ_msgs, msg, Contacts)() {
  return &::champ_msgs::msg::rosidl_typesupport_introspection_cpp::Contacts_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
