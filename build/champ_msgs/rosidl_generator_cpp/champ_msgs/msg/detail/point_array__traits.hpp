// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from champ_msgs:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_
#define CHAMP_MSGS__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "champ_msgs/msg/detail/point_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'lf'
// Member 'rf'
// Member 'lh'
// Member 'rh'
#include "champ_msgs/msg/detail/point__traits.hpp"

namespace champ_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PointArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: lf
  {
    out << "lf: ";
    to_flow_style_yaml(msg.lf, out);
    out << ", ";
  }

  // member: rf
  {
    out << "rf: ";
    to_flow_style_yaml(msg.rf, out);
    out << ", ";
  }

  // member: lh
  {
    out << "lh: ";
    to_flow_style_yaml(msg.lh, out);
    out << ", ";
  }

  // member: rh
  {
    out << "rh: ";
    to_flow_style_yaml(msg.rh, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PointArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: lf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lf:\n";
    to_block_style_yaml(msg.lf, out, indentation + 2);
  }

  // member: rf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rf:\n";
    to_block_style_yaml(msg.rf, out, indentation + 2);
  }

  // member: lh
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lh:\n";
    to_block_style_yaml(msg.lh, out, indentation + 2);
  }

  // member: rh
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rh:\n";
    to_block_style_yaml(msg.rh, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PointArray & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace champ_msgs

namespace rosidl_generator_traits
{

[[deprecated("use champ_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const champ_msgs::msg::PointArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  champ_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use champ_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const champ_msgs::msg::PointArray & msg)
{
  return champ_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<champ_msgs::msg::PointArray>()
{
  return "champ_msgs::msg::PointArray";
}

template<>
inline const char * name<champ_msgs::msg::PointArray>()
{
  return "champ_msgs/msg/PointArray";
}

template<>
struct has_fixed_size<champ_msgs::msg::PointArray>
  : std::integral_constant<bool, has_fixed_size<champ_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<champ_msgs::msg::PointArray>
  : std::integral_constant<bool, has_bounded_size<champ_msgs::msg::Point>::value> {};

template<>
struct is_message<champ_msgs::msg::PointArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CHAMP_MSGS__MSG__DETAIL__POINT_ARRAY__TRAITS_HPP_
