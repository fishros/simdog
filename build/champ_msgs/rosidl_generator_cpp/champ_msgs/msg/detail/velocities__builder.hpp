// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from champ_msgs:msg/Velocities.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__VELOCITIES__BUILDER_HPP_
#define CHAMP_MSGS__MSG__DETAIL__VELOCITIES__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "champ_msgs/msg/detail/velocities__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace champ_msgs
{

namespace msg
{

namespace builder
{

class Init_Velocities_angular_z
{
public:
  explicit Init_Velocities_angular_z(::champ_msgs::msg::Velocities & msg)
  : msg_(msg)
  {}
  ::champ_msgs::msg::Velocities angular_z(::champ_msgs::msg::Velocities::_angular_z_type arg)
  {
    msg_.angular_z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::champ_msgs::msg::Velocities msg_;
};

class Init_Velocities_linear_y
{
public:
  explicit Init_Velocities_linear_y(::champ_msgs::msg::Velocities & msg)
  : msg_(msg)
  {}
  Init_Velocities_angular_z linear_y(::champ_msgs::msg::Velocities::_linear_y_type arg)
  {
    msg_.linear_y = std::move(arg);
    return Init_Velocities_angular_z(msg_);
  }

private:
  ::champ_msgs::msg::Velocities msg_;
};

class Init_Velocities_linear_x
{
public:
  Init_Velocities_linear_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Velocities_linear_y linear_x(::champ_msgs::msg::Velocities::_linear_x_type arg)
  {
    msg_.linear_x = std::move(arg);
    return Init_Velocities_linear_y(msg_);
  }

private:
  ::champ_msgs::msg::Velocities msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::champ_msgs::msg::Velocities>()
{
  return champ_msgs::msg::builder::Init_Velocities_linear_x();
}

}  // namespace champ_msgs

#endif  // CHAMP_MSGS__MSG__DETAIL__VELOCITIES__BUILDER_HPP_
