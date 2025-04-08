// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from champ_msgs:msg/Imu.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__IMU__BUILDER_HPP_
#define CHAMP_MSGS__MSG__DETAIL__IMU__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "champ_msgs/msg/detail/imu__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace champ_msgs
{

namespace msg
{

namespace builder
{

class Init_Imu_magnetic_field
{
public:
  explicit Init_Imu_magnetic_field(::champ_msgs::msg::Imu & msg)
  : msg_(msg)
  {}
  ::champ_msgs::msg::Imu magnetic_field(::champ_msgs::msg::Imu::_magnetic_field_type arg)
  {
    msg_.magnetic_field = std::move(arg);
    return std::move(msg_);
  }

private:
  ::champ_msgs::msg::Imu msg_;
};

class Init_Imu_angular_velocity
{
public:
  explicit Init_Imu_angular_velocity(::champ_msgs::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_magnetic_field angular_velocity(::champ_msgs::msg::Imu::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_Imu_magnetic_field(msg_);
  }

private:
  ::champ_msgs::msg::Imu msg_;
};

class Init_Imu_linear_acceleration
{
public:
  explicit Init_Imu_linear_acceleration(::champ_msgs::msg::Imu & msg)
  : msg_(msg)
  {}
  Init_Imu_angular_velocity linear_acceleration(::champ_msgs::msg::Imu::_linear_acceleration_type arg)
  {
    msg_.linear_acceleration = std::move(arg);
    return Init_Imu_angular_velocity(msg_);
  }

private:
  ::champ_msgs::msg::Imu msg_;
};

class Init_Imu_orientation
{
public:
  Init_Imu_orientation()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Imu_linear_acceleration orientation(::champ_msgs::msg::Imu::_orientation_type arg)
  {
    msg_.orientation = std::move(arg);
    return Init_Imu_linear_acceleration(msg_);
  }

private:
  ::champ_msgs::msg::Imu msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::champ_msgs::msg::Imu>()
{
  return champ_msgs::msg::builder::Init_Imu_orientation();
}

}  // namespace champ_msgs

#endif  // CHAMP_MSGS__MSG__DETAIL__IMU__BUILDER_HPP_
