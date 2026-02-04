// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from status_interfaces:msg/SystemStatus.idl
// generated code does not contain a copyright notice

#include "status_interfaces/msg/detail/system_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_status_interfaces
const rosidl_type_hash_t *
status_interfaces__msg__SystemStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x07, 0x0a, 0xe5, 0xd0, 0x58, 0x03, 0x15, 0xeb,
      0xa3, 0x51, 0xf4, 0xa1, 0xca, 0x70, 0x38, 0x85,
      0xdf, 0xb6, 0xf6, 0xca, 0x75, 0xf0, 0x13, 0xaa,
      0x70, 0x7e, 0xc5, 0x60, 0x0e, 0xdc, 0x59, 0xee,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
#endif

static char status_interfaces__msg__SystemStatus__TYPE_NAME[] = "status_interfaces/msg/SystemStatus";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";

// Define type names, field names, and default values
static char status_interfaces__msg__SystemStatus__FIELD_NAME__stamp[] = "stamp";
static char status_interfaces__msg__SystemStatus__FIELD_NAME__host_name[] = "host_name";
static char status_interfaces__msg__SystemStatus__FIELD_NAME__cpu_percent[] = "cpu_percent";
static char status_interfaces__msg__SystemStatus__FIELD_NAME__memory_percent[] = "memory_percent";
static char status_interfaces__msg__SystemStatus__FIELD_NAME__memory_total[] = "memory_total";
static char status_interfaces__msg__SystemStatus__FIELD_NAME__memory_available[] = "memory_available";
static char status_interfaces__msg__SystemStatus__FIELD_NAME__net_sent[] = "net_sent";
static char status_interfaces__msg__SystemStatus__FIELD_NAME__net_recv[] = "net_recv";

static rosidl_runtime_c__type_description__Field status_interfaces__msg__SystemStatus__FIELDS[] = {
  {
    {status_interfaces__msg__SystemStatus__FIELD_NAME__stamp, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {status_interfaces__msg__SystemStatus__FIELD_NAME__host_name, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {status_interfaces__msg__SystemStatus__FIELD_NAME__cpu_percent, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {status_interfaces__msg__SystemStatus__FIELD_NAME__memory_percent, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {status_interfaces__msg__SystemStatus__FIELD_NAME__memory_total, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {status_interfaces__msg__SystemStatus__FIELD_NAME__memory_available, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {status_interfaces__msg__SystemStatus__FIELD_NAME__net_sent, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {status_interfaces__msg__SystemStatus__FIELD_NAME__net_recv, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription status_interfaces__msg__SystemStatus__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
status_interfaces__msg__SystemStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {status_interfaces__msg__SystemStatus__TYPE_NAME, 34, 34},
      {status_interfaces__msg__SystemStatus__FIELDS, 8, 8},
    },
    {status_interfaces__msg__SystemStatus__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "builtin_interfaces/Time stamp # \\xe8\\xae\\xb0\\xe5\\xbd\\x95\\xe6\\x97\\xb6\\xe9\\x97\\xb4\\xe6\\x88\\xb3\n"
  "string host_name # \\xe4\\xb8\\xbb\\xe6\\x9c\\xba\\xe5\\x90\\x8d\\xe5\\xad\\x97\n"
  "float32 cpu_percent # CPU\\xe4\\xbd\\xbf\\xe7\\x94\\xa8\\xe7\\x8e\\x87\n"
  "float32 memory_percent # \\xe5\\x86\\x85\\xe5\\xad\\x98\\xe4\\xbd\\xbf\\xe7\\x94\\xa8\\xe7\\x8e\\x87\n"
  "float32 memory_total # \\xe5\\x86\\x85\\xe5\\xad\\x98\\xe6\\x80\\xbb\\xe5\\xae\\xb9\\xe9\\x87\\x8f\n"
  "float32 memory_available # \\xe5\\x86\\x85\\xe5\\xad\\x98\\xe5\\x8f\\xaf\\xe7\\x94\\xa8\\xe5\\xae\\xb9\\xe9\\x87\\x8f\n"
  "float64 net_sent # \\xe7\\xbd\\x91\\xe7\\xbb\\x9c\\xe5\\x8f\\x91\\xe9\\x80\\x81\\xe6\\x95\\xb0\\xe6\\x8d\\xae\\xe6\\x80\\xbb\\xe9\\x87\\x8f 1MB = 8MB\n"
  "float64 net_recv # \\xe7\\xbd\\x91\\xe7\\xbb\\x9c\\xe6\\x8e\\xa5\\xe6\\x94\\xb6\\xe6\\x95\\xb0\\xe6\\x8d\\xae\\xe6\\x80\\xbb\\xe9\\x87\\x8f MB";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
status_interfaces__msg__SystemStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {status_interfaces__msg__SystemStatus__TYPE_NAME, 34, 34},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 254, 254},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
status_interfaces__msg__SystemStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *status_interfaces__msg__SystemStatus__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
