// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/SocketMsg.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "interfaces/msg/socket_msg.h"


#ifndef INTERFACES__MSG__DETAIL__SOCKET_MSG__STRUCT_H_
#define INTERFACES__MSG__DETAIL__SOCKET_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/SocketMsg in the package interfaces.
typedef struct interfaces__msg__SocketMsg
{
  double timestamp;
  rosidl_runtime_c__String data;
} interfaces__msg__SocketMsg;

// Struct for a sequence of interfaces__msg__SocketMsg.
typedef struct interfaces__msg__SocketMsg__Sequence
{
  interfaces__msg__SocketMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__SocketMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__SOCKET_MSG__STRUCT_H_
