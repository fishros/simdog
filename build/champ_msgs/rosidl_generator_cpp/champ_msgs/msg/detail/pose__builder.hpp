// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from champ_msgs:msg/Pose.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_
#define CHAMP_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "champ_msgs/msg/detail/pose__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace champ_msgs
{

namespace msg
{

namespace builder
{

class Init_Pose_yaw
{
public:
  explicit Init_Pose_yaw(::champ_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  ::champ_msgs::msg::Pose yaw(::champ_msgs::msg::Pose::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::champ_msgs::msg::Pose msg_;
};

class Init_Pose_pitch
{
public:
  explicit Init_Pose_pitch(::champ_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_yaw pitch(::champ_msgs::msg::Pose::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_Pose_yaw(msg_);
  }

private:
  ::champ_msgs::msg::Pose msg_;
};

class Init_Pose_roll
{
public:
  explicit Init_Pose_roll(::champ_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_pitch roll(::champ_msgs::msg::Pose::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_Pose_pitch(msg_);
  }

private:
  ::champ_msgs::msg::Pose msg_;
};

class Init_Pose_z
{
public:
  explicit Init_Pose_z(::champ_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_roll z(::champ_msgs::msg::Pose::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Pose_roll(msg_);
  }

private:
  ::champ_msgs::msg::Pose msg_;
};

class Init_Pose_y
{
public:
  explicit Init_Pose_y(::champ_msgs::msg::Pose & msg)
  : msg_(msg)
  {}
  Init_Pose_z y(::champ_msgs::msg::Pose::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Pose_z(msg_);
  }

private:
  ::champ_msgs::msg::Pose msg_;
};

class Init_Pose_x
{
public:
  Init_Pose_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Pose_y x(::champ_msgs::msg::Pose::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Pose_y(msg_);
  }

private:
  ::champ_msgs::msg::Pose msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::champ_msgs::msg::Pose>()
{
  return champ_msgs::msg::builder::Init_Pose_x();
}

}  // namespace champ_msgs

#endif  // CHAMP_MSGS__MSG__DETAIL__POSE__BUILDER_HPP_
