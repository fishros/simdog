// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from champ_msgs:msg/Contacts.idl
// generated code does not contain a copyright notice

#ifndef CHAMP_MSGS__MSG__DETAIL__CONTACTS__BUILDER_HPP_
#define CHAMP_MSGS__MSG__DETAIL__CONTACTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "champ_msgs/msg/detail/contacts__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace champ_msgs
{

namespace msg
{

namespace builder
{

class Init_Contacts_contacts
{
public:
  Init_Contacts_contacts()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::champ_msgs::msg::Contacts contacts(::champ_msgs::msg::Contacts::_contacts_type arg)
  {
    msg_.contacts = std::move(arg);
    return std::move(msg_);
  }

private:
  ::champ_msgs::msg::Contacts msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::champ_msgs::msg::Contacts>()
{
  return champ_msgs::msg::builder::Init_Contacts_contacts();
}

}  // namespace champ_msgs

#endif  // CHAMP_MSGS__MSG__DETAIL__CONTACTS__BUILDER_HPP_
