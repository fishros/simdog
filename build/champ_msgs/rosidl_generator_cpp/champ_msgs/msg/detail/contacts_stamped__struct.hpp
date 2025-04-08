// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from champ_msgs:msg/ContactsStamped.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__CONTACTS_STAMPED__STRUCT_HPP_
#define CHAMP_MSGS__MSG__DETAIL__CONTACTS_STAMPED__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__champ_msgs__msg__ContactsStamped __attribute__((deprecated))
#else
# define DEPRECATED__champ_msgs__msg__ContactsStamped __declspec(deprecated)
#endif

namespace champ_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ContactsStamped_
{
  using Type = ContactsStamped_<ContainerAllocator>;

  explicit ContactsStamped_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit ContactsStamped_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _contacts_type =
    std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>>;
  _contacts_type contacts;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__contacts(
    const std::vector<bool, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<bool>> & _arg)
  {
    this->contacts = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    champ_msgs::msg::ContactsStamped_<ContainerAllocator> *;
  using ConstRawPtr =
    const champ_msgs::msg::ContactsStamped_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<champ_msgs::msg::ContactsStamped_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<champ_msgs::msg::ContactsStamped_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      champ_msgs::msg::ContactsStamped_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<champ_msgs::msg::ContactsStamped_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      champ_msgs::msg::ContactsStamped_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<champ_msgs::msg::ContactsStamped_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<champ_msgs::msg::ContactsStamped_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<champ_msgs::msg::ContactsStamped_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__champ_msgs__msg__ContactsStamped
    std::shared_ptr<champ_msgs::msg::ContactsStamped_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__champ_msgs__msg__ContactsStamped
    std::shared_ptr<champ_msgs::msg::ContactsStamped_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ContactsStamped_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->contacts != other.contacts) {
      return false;
    }
    return true;
  }
  bool operator!=(const ContactsStamped_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ContactsStamped_

// alias to use template instance with default allocator
using ContactsStamped =
  champ_msgs::msg::ContactsStamped_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace champ_msgs

#endif  // CHAMP_MSGS__MSG__DETAIL__CONTACTS_STAMPED__STRUCT_HPP_
