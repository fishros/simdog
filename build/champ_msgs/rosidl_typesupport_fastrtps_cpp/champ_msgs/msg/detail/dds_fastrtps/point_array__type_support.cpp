// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from champ_msgs:msg/PointArray.idl
// generated code does not contain a copyright notice
#include "champ_msgs/msg/detail/point_array__rosidl_typesupport_fastrtps_cpp.hpp"
#include "champ_msgs/msg/detail/point_array__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace champ_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const champ_msgs::msg::Point &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  champ_msgs::msg::Point &);
size_t get_serialized_size(
  const champ_msgs::msg::Point &,
  size_t current_alignment);
size_t
max_serialized_size_Point(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace champ_msgs

// functions for champ_msgs::msg::Point already declared above

// functions for champ_msgs::msg::Point already declared above

// functions for champ_msgs::msg::Point already declared above


namespace champ_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_champ_msgs
cdr_serialize(
  const champ_msgs::msg::PointArray & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: lf
  champ_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.lf,
    cdr);
  // Member: rf
  champ_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.rf,
    cdr);
  // Member: lh
  champ_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.lh,
    cdr);
  // Member: rh
  champ_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.rh,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_champ_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  champ_msgs::msg::PointArray & ros_message)
{
  // Member: lf
  champ_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.lf);

  // Member: rf
  champ_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.rf);

  // Member: lh
  champ_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.lh);

  // Member: rh
  champ_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.rh);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_champ_msgs
get_serialized_size(
  const champ_msgs::msg::PointArray & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: lf

  current_alignment +=
    champ_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.lf, current_alignment);
  // Member: rf

  current_alignment +=
    champ_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.rf, current_alignment);
  // Member: lh

  current_alignment +=
    champ_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.lh, current_alignment);
  // Member: rh

  current_alignment +=
    champ_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.rh, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_champ_msgs
max_serialized_size_PointArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: lf
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        champ_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: rf
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        champ_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: lh
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        champ_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: rh
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        champ_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Point(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = champ_msgs::msg::PointArray;
    is_plain =
      (
      offsetof(DataType, rh) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _PointArray__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const champ_msgs::msg::PointArray *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PointArray__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<champ_msgs::msg::PointArray *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PointArray__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const champ_msgs::msg::PointArray *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PointArray__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_PointArray(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _PointArray__callbacks = {
  "champ_msgs::msg",
  "PointArray",
  _PointArray__cdr_serialize,
  _PointArray__cdr_deserialize,
  _PointArray__get_serialized_size,
  _PointArray__max_serialized_size
};

static rosidl_message_type_support_t _PointArray__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PointArray__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace champ_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_champ_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<champ_msgs::msg::PointArray>()
{
  return &champ_msgs::msg::typesupport_fastrtps_cpp::_PointArray__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, champ_msgs, msg, PointArray)() {
  return &champ_msgs::msg::typesupport_fastrtps_cpp::_PointArray__handle;
}

#ifdef __cplusplus
}
#endif
