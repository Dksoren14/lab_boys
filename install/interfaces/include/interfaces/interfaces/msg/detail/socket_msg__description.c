// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from interfaces:msg/SocketMsg.idl
// generated code does not contain a copyright notice

#include "interfaces/msg/detail/socket_msg__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_interfaces
const rosidl_type_hash_t *
interfaces__msg__SocketMsg__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3b, 0x54, 0x81, 0xb7, 0x66, 0xf7, 0x36, 0x65,
      0xd0, 0x21, 0x4d, 0x7f, 0x07, 0x20, 0xdf, 0x31,
      0x87, 0x9b, 0xc8, 0x89, 0xaf, 0x80, 0x40, 0xd9,
      0x40, 0x06, 0x91, 0x8a, 0xbe, 0x40, 0xd1, 0x09,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char interfaces__msg__SocketMsg__TYPE_NAME[] = "interfaces/msg/SocketMsg";

// Define type names, field names, and default values
static char interfaces__msg__SocketMsg__FIELD_NAME__timestamp[] = "timestamp";
static char interfaces__msg__SocketMsg__FIELD_NAME__data[] = "data";

static rosidl_runtime_c__type_description__Field interfaces__msg__SocketMsg__FIELDS[] = {
  {
    {interfaces__msg__SocketMsg__FIELD_NAME__timestamp, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {interfaces__msg__SocketMsg__FIELD_NAME__data, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
interfaces__msg__SocketMsg__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {interfaces__msg__SocketMsg__TYPE_NAME, 24, 24},
      {interfaces__msg__SocketMsg__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 timestamp\n"
  "string data";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
interfaces__msg__SocketMsg__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {interfaces__msg__SocketMsg__TYPE_NAME, 24, 24},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 29, 29},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
interfaces__msg__SocketMsg__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *interfaces__msg__SocketMsg__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
