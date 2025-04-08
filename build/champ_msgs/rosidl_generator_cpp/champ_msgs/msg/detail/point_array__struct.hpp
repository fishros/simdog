// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from champ_msgs:msg/PointArray.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__POINT_ARRAY__STRUCT_HPP_
#define CHAMP_MSGS__MSG__DETAIL__POINT_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'lf'
// Member 'rf'
// Member 'lh'
// Member 'rh'
#include "champ_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__champ_msgs__msg__PointArray __attribute__((deprecated))
#else
# define DEPRECATED__champ_msgs__msg__PointArray __declspec(deprecated)
#endif

namespace champ_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PointArray_
{
  using Type = PointArray_<ContainerAllocator>;

  explicit PointArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : lf(_init),
    rf(_init),
    lh(_init),
    rh(_init)
  {
    (void)_init;
  }

  explicit PointArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : lf(_alloc, _init),
    rf(_alloc, _init),
    lh(_alloc, _init),
    rh(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _lf_type =
    champ_msgs::msg::Point_<ContainerAllocator>;
  _lf_type lf;
  using _rf_type =
    champ_msgs::msg::Point_<ContainerAllocator>;
  _rf_type rf;
  using _lh_type =
    champ_msgs::msg::Point_<ContainerAllocator>;
  _lh_type lh;
  using _rh_type =
    champ_msgs::msg::Point_<ContainerAllocator>;
  _rh_type rh;

  // setters for named parameter idiom
  Type & set__lf(
    const champ_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->lf = _arg;
    return *this;
  }
  Type & set__rf(
    const champ_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->rf = _arg;
    return *this;
  }
  Type & set__lh(
    const champ_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->lh = _arg;
    return *this;
  }
  Type & set__rh(
    const champ_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->rh = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    champ_msgs::msg::PointArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const champ_msgs::msg::PointArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<champ_msgs::msg::PointArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<champ_msgs::msg::PointArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      champ_msgs::msg::PointArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<champ_msgs::msg::PointArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      champ_msgs::msg::PointArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<champ_msgs::msg::PointArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<champ_msgs::msg::PointArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<champ_msgs::msg::PointArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__champ_msgs__msg__PointArray
    std::shared_ptr<champ_msgs::msg::PointArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__champ_msgs__msg__PointArray
    std::shared_ptr<champ_msgs::msg::PointArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PointArray_ & other) const
  {
    if (this->lf != other.lf) {
      return false;
    }
    if (this->rf != other.rf) {
      return false;
    }
    if (this->lh != other.lh) {
      return false;
    }
    if (this->rh != other.rh) {
      return false;
    }
    return true;
  }
  bool operator!=(const PointArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PointArray_

// alias to use template instance with default allocator
using PointArray =
  champ_msgs::msg::PointArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace champ_msgs

#endif  // CHAMP_MSGS__MSG__DETAIL__POINT_ARRAY__STRUCT_HPP_
