// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from champ_msgs:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__POINT_ARRAY__BUILDER_HPP_
#define CHAMP_MSGS__MSG__DETAIL__POINT_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "champ_msgs/msg/detail/point_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace champ_msgs
{

namespace msg
{

namespace builder
{

class Init_PointArray_rh
{
public:
  explicit Init_PointArray_rh(::champ_msgs::msg::PointArray & msg)
  : msg_(msg)
  {}
  ::champ_msgs::msg::PointArray rh(::champ_msgs::msg::PointArray::_rh_type arg)
  {
    msg_.rh = std::move(arg);
    return std::move(msg_);
  }

private:
  ::champ_msgs::msg::PointArray msg_;
};

class Init_PointArray_lh
{
public:
  explicit Init_PointArray_lh(::champ_msgs::msg::PointArray & msg)
  : msg_(msg)
  {}
  Init_PointArray_rh lh(::champ_msgs::msg::PointArray::_lh_type arg)
  {
    msg_.lh = std::move(arg);
    return Init_PointArray_rh(msg_);
  }

private:
  ::champ_msgs::msg::PointArray msg_;
};

class Init_PointArray_rf
{
public:
  explicit Init_PointArray_rf(::champ_msgs::msg::PointArray & msg)
  : msg_(msg)
  {}
  Init_PointArray_lh rf(::champ_msgs::msg::PointArray::_rf_type arg)
  {
    msg_.rf = std::move(arg);
    return Init_PointArray_lh(msg_);
  }

private:
  ::champ_msgs::msg::PointArray msg_;
};

class Init_PointArray_lf
{
public:
  Init_PointArray_lf()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PointArray_rf lf(::champ_msgs::msg::PointArray::_lf_type arg)
  {
    msg_.lf = std::move(arg);
    return Init_PointArray_rf(msg_);
  }

private:
  ::champ_msgs::msg::PointArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::champ_msgs::msg::PointArray>()
{
  return champ_msgs::msg::builder::Init_PointArray_lf();
}

}  // namespace champ_msgs

#endif  // CHAMP_MSGS__MSG__DETAIL__POINT_ARRAY__BUILDER_HPP_
