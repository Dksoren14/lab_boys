// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:msg/SocketMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/socket_msg.hpp"


#ifndef INTERFACES__MSG__DETAIL__SOCKET_MSG__TRAITS_HPP_
#define INTERFACES__MSG__DETAIL__SOCKET_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/msg/detail/socket_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SocketMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SocketMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SocketMsg & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::msg::SocketMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::msg::SocketMsg & msg)
{
  return interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::msg::SocketMsg>()
{
  return "interfaces::msg::SocketMsg";
}

template<>
inline const char * name<interfaces::msg::SocketMsg>()
{
  return "interfaces/msg/SocketMsg";
}

template<>
struct has_fixed_size<interfaces::msg::SocketMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::msg::SocketMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::msg::SocketMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__MSG__DETAIL__SOCKET_MSG__TRAITS_HPP_
