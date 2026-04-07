// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/SocketMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/socket_msg.hpp"


#ifndef INTERFACES__MSG__DETAIL__SOCKET_MSG__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__SOCKET_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/socket_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_SocketMsg_data
{
public:
  explicit Init_SocketMsg_data(::interfaces::msg::SocketMsg & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::SocketMsg data(::interfaces::msg::SocketMsg::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::SocketMsg msg_;
};

class Init_SocketMsg_timestamp
{
public:
  Init_SocketMsg_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SocketMsg_data timestamp(::interfaces::msg::SocketMsg::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_SocketMsg_data(msg_);
  }

private:
  ::interfaces::msg::SocketMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::SocketMsg>()
{
  return interfaces::msg::builder::Init_SocketMsg_timestamp();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__SOCKET_MSG__BUILDER_HPP_
