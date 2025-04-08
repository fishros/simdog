// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from champ_msgs:msg/Velocities.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__VELOCITIES__STRUCT_HPP_
#define CHAMP_MSGS__MSG__DETAIL__VELOCITIES__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__champ_msgs__msg__Velocities __attribute__((deprecated))
#else
# define DEPRECATED__champ_msgs__msg__Velocities __declspec(deprecated)
#endif

namespace champ_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Velocities_
{
  using Type = Velocities_<ContainerAllocator>;

  explicit Velocities_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_x = 0.0f;
      this->linear_y = 0.0f;
      this->angular_z = 0.0f;
    }
  }

  explicit Velocities_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_x = 0.0f;
      this->linear_y = 0.0f;
      this->angular_z = 0.0f;
    }
  }

  // field types and members
  using _linear_x_type =
    float;
  _linear_x_type linear_x;
  using _linear_y_type =
    float;
  _linear_y_type linear_y;
  using _angular_z_type =
    float;
  _angular_z_type angular_z;

  // setters for named parameter idiom
  Type & set__linear_x(
    const float & _arg)
  {
    this->linear_x = _arg;
    return *this;
  }
  Type & set__linear_y(
    const float & _arg)
  {
    this->linear_y = _arg;
    return *this;
  }
  Type & set__angular_z(
    const float & _arg)
  {
    this->angular_z = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    champ_msgs::msg::Velocities_<ContainerAllocator> *;
  using ConstRawPtr =
    const champ_msgs::msg::Velocities_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<champ_msgs::msg::Velocities_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<champ_msgs::msg::Velocities_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      champ_msgs::msg::Velocities_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<champ_msgs::msg::Velocities_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      champ_msgs::msg::Velocities_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<champ_msgs::msg::Velocities_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<champ_msgs::msg::Velocities_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<champ_msgs::msg::Velocities_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__champ_msgs__msg__Velocities
    std::shared_ptr<champ_msgs::msg::Velocities_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__champ_msgs__msg__Velocities
    std::shared_ptr<champ_msgs::msg::Velocities_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Velocities_ & other) const
  {
    if (this->linear_x != other.linear_x) {
      return false;
    }
    if (this->linear_y != other.linear_y) {
      return false;
    }
    if (this->angular_z != other.angular_z) {
      return false;
    }
    return true;
  }
  bool operator!=(const Velocities_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Velocities_

// alias to use template instance with default allocator
using Velocities =
  champ_msgs::msg::Velocities_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace champ_msgs

#endif  // CHAMP_MSGS__MSG__DETAIL__VELOCITIES__STRUCT_HPP_
