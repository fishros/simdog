// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from champ_msgs:msg/ContactsStamped.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__CONTACTS_STAMPED__BUILDER_HPP_
#define CHAMP_MSGS__MSG__DETAIL__CONTACTS_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "champ_msgs/msg/detail/contacts_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace champ_msgs
{

namespace msg
{

namespace builder
{

class Init_ContactsStamped_contacts
{
public:
  explicit Init_ContactsStamped_contacts(::champ_msgs::msg::ContactsStamped & msg)
  : msg_(msg)
  {}
  ::champ_msgs::msg::ContactsStamped contacts(::champ_msgs::msg::ContactsStamped::_contacts_type arg)
  {
    msg_.contacts = std::move(arg);
    return std::move(msg_);
  }

private:
  ::champ_msgs::msg::ContactsStamped msg_;
};

class Init_ContactsStamped_header
{
public:
  Init_ContactsStamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ContactsStamped_contacts header(::champ_msgs::msg::ContactsStamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ContactsStamped_contacts(msg_);
  }

private:
  ::champ_msgs::msg::ContactsStamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::champ_msgs::msg::ContactsStamped>()
{
  return champ_msgs::msg::builder::Init_ContactsStamped_header();
}

}  // namespace champ_msgs

#endif  // CHAMP_MSGS__MSG__DETAIL__CONTACTS_STAMPED__BUILDER_HPP_
