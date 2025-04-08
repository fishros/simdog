// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from champ_msgs:msg/PID.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__PID__BUILDER_HPP_
#define CHAMP_MSGS__MSG__DETAIL__PID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "champ_msgs/msg/detail/pid__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace champ_msgs
{

namespace msg
{

namespace builder
{

class Init_PID_i
{
public:
  explicit Init_PID_i(::champ_msgs::msg::PID & msg)
  : msg_(msg)
  {}
  ::champ_msgs::msg::PID i(::champ_msgs::msg::PID::_i_type arg)
  {
    msg_.i = std::move(arg);
    return std::move(msg_);
  }

private:
  ::champ_msgs::msg::PID msg_;
};

class Init_PID_d
{
public:
  explicit Init_PID_d(::champ_msgs::msg::PID & msg)
  : msg_(msg)
  {}
  Init_PID_i d(::champ_msgs::msg::PID::_d_type arg)
  {
    msg_.d = std::move(arg);
    return Init_PID_i(msg_);
  }

private:
  ::champ_msgs::msg::PID msg_;
};

class Init_PID_p
{
public:
  Init_PID_p()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PID_d p(::champ_msgs::msg::PID::_p_type arg)
  {
    msg_.p = std::move(arg);
    return Init_PID_d(msg_);
  }

private:
  ::champ_msgs::msg::PID msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::champ_msgs::msg::PID>()
{
  return champ_msgs::msg::builder::Init_PID_p();
}

}  // namespace champ_msgs

#endif  // CHAMP_MSGS__MSG__DETAIL__PID__BUILDER_HPP_
