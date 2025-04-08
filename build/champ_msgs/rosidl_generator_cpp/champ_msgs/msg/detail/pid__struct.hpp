// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from champ_msgs:msg/PID.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__PID__STRUCT_HPP_
#define CHAMP_MSGS__MSG__DETAIL__PID__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__champ_msgs__msg__PID __attribute__((deprecated))
#else
# define DEPRECATED__champ_msgs__msg__PID __declspec(deprecated)
#endif

namespace champ_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PID_
{
  using Type = PID_<ContainerAllocator>;

  explicit PID_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->p = 0.0f;
      this->d = 0.0f;
      this->i = 0.0f;
    }
  }

  explicit PID_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->p = 0.0f;
      this->d = 0.0f;
      this->i = 0.0f;
    }
  }

  // field types and members
  using _p_type =
    float;
  _p_type p;
  using _d_type =
    float;
  _d_type d;
  using _i_type =
    float;
  _i_type i;

  // setters for named parameter idiom
  Type & set__p(
    const float & _arg)
  {
    this->p = _arg;
    return *this;
  }
  Type & set__d(
    const float & _arg)
  {
    this->d = _arg;
    return *this;
  }
  Type & set__i(
    const float & _arg)
  {
    this->i = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    champ_msgs::msg::PID_<ContainerAllocator> *;
  using ConstRawPtr =
    const champ_msgs::msg::PID_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<champ_msgs::msg::PID_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<champ_msgs::msg::PID_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      champ_msgs::msg::PID_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<champ_msgs::msg::PID_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      champ_msgs::msg::PID_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<champ_msgs::msg::PID_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<champ_msgs::msg::PID_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<champ_msgs::msg::PID_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__champ_msgs__msg__PID
    std::shared_ptr<champ_msgs::msg::PID_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__champ_msgs__msg__PID
    std::shared_ptr<champ_msgs::msg::PID_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PID_ & other) const
  {
    if (this->p != other.p) {
      return false;
    }
    if (this->d != other.d) {
      return false;
    }
    if (this->i != other.i) {
      return false;
    }
    return true;
  }
  bool operator!=(const PID_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PID_

// alias to use template instance with default allocator
using PID =
  champ_msgs::msg::PID_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace champ_msgs

#endif  // CHAMP_MSGS__MSG__DETAIL__PID__STRUCT_HPP_
