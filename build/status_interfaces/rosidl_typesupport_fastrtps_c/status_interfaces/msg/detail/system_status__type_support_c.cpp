// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from status_interfaces:msg/SystemStatus.idl
// generated code does not contain a copyright notice
#include "status_interfaces/msg/detail/system_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "status_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "status_interfaces/msg/detail/system_status__struct.h"
#include "status_interfaces/msg/detail/system_status__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "builtin_interfaces/msg/detail/time__functions.h"  // stamp
#include "rosidl_runtime_c/string.h"  // host_name
#include "rosidl_runtime_c/string_functions.h"  // host_name

// forward declare type support functions

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_status_interfaces
bool cdr_serialize_builtin_interfaces__msg__Time(
  const builtin_interfaces__msg__Time * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_status_interfaces
bool cdr_deserialize_builtin_interfaces__msg__Time(
  eprosima::fastcdr::Cdr & cdr,
  builtin_interfaces__msg__Time * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_status_interfaces
size_t get_serialized_size_builtin_interfaces__msg__Time(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_status_interfaces
size_t max_serialized_size_builtin_interfaces__msg__Time(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_status_interfaces
bool cdr_serialize_key_builtin_interfaces__msg__Time(
  const builtin_interfaces__msg__Time * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_status_interfaces
size_t get_serialized_size_key_builtin_interfaces__msg__Time(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_status_interfaces
size_t max_serialized_size_key_builtin_interfaces__msg__Time(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_status_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, builtin_interfaces, msg, Time)();


using _SystemStatus__ros_msg_type = status_interfaces__msg__SystemStatus;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_status_interfaces
bool cdr_serialize_status_interfaces__msg__SystemStatus(
  const status_interfaces__msg__SystemStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: stamp
  {
    cdr_serialize_builtin_interfaces__msg__Time(
      &ros_message->stamp, cdr);
  }

  // Field name: host_name
  {
    const rosidl_runtime_c__String * str = &ros_message->host_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: cpu_percent
  {
    cdr << ros_message->cpu_percent;
  }

  // Field name: memory_percent
  {
    cdr << ros_message->memory_percent;
  }

  // Field name: memory_total
  {
    cdr << ros_message->memory_total;
  }

  // Field name: memory_available
  {
    cdr << ros_message->memory_available;
  }

  // Field name: net_sent
  {
    cdr << ros_message->net_sent;
  }

  // Field name: net_recv
  {
    cdr << ros_message->net_recv;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_status_interfaces
bool cdr_deserialize_status_interfaces__msg__SystemStatus(
  eprosima::fastcdr::Cdr & cdr,
  status_interfaces__msg__SystemStatus * ros_message)
{
  // Field name: stamp
  {
    cdr_deserialize_builtin_interfaces__msg__Time(cdr, &ros_message->stamp);
  }

  // Field name: host_name
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->host_name.data) {
      rosidl_runtime_c__String__init(&ros_message->host_name);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->host_name,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'host_name'\n");
      return false;
    }
  }

  // Field name: cpu_percent
  {
    cdr >> ros_message->cpu_percent;
  }

  // Field name: memory_percent
  {
    cdr >> ros_message->memory_percent;
  }

  // Field name: memory_total
  {
    cdr >> ros_message->memory_total;
  }

  // Field name: memory_available
  {
    cdr >> ros_message->memory_available;
  }

  // Field name: net_sent
  {
    cdr >> ros_message->net_sent;
  }

  // Field name: net_recv
  {
    cdr >> ros_message->net_recv;
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_status_interfaces
size_t get_serialized_size_status_interfaces__msg__SystemStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SystemStatus__ros_msg_type * ros_message = static_cast<const _SystemStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: stamp
  current_alignment += get_serialized_size_builtin_interfaces__msg__Time(
    &(ros_message->stamp), current_alignment);

  // Field name: host_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->host_name.size + 1);

  // Field name: cpu_percent
  {
    size_t item_size = sizeof(ros_message->cpu_percent);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: memory_percent
  {
    size_t item_size = sizeof(ros_message->memory_percent);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: memory_total
  {
    size_t item_size = sizeof(ros_message->memory_total);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: memory_available
  {
    size_t item_size = sizeof(ros_message->memory_available);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: net_sent
  {
    size_t item_size = sizeof(ros_message->net_sent);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: net_recv
  {
    size_t item_size = sizeof(ros_message->net_recv);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_status_interfaces
size_t max_serialized_size_status_interfaces__msg__SystemStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // Field name: stamp
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_builtin_interfaces__msg__Time(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: host_name
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: cpu_percent
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: memory_percent
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: memory_total
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: memory_available
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: net_sent
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: net_recv
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = status_interfaces__msg__SystemStatus;
    is_plain =
      (
      offsetof(DataType, net_recv) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_status_interfaces
bool cdr_serialize_key_status_interfaces__msg__SystemStatus(
  const status_interfaces__msg__SystemStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: stamp
  {
    cdr_serialize_key_builtin_interfaces__msg__Time(
      &ros_message->stamp, cdr);
  }

  // Field name: host_name
  {
    const rosidl_runtime_c__String * str = &ros_message->host_name;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: cpu_percent
  {
    cdr << ros_message->cpu_percent;
  }

  // Field name: memory_percent
  {
    cdr << ros_message->memory_percent;
  }

  // Field name: memory_total
  {
    cdr << ros_message->memory_total;
  }

  // Field name: memory_available
  {
    cdr << ros_message->memory_available;
  }

  // Field name: net_sent
  {
    cdr << ros_message->net_sent;
  }

  // Field name: net_recv
  {
    cdr << ros_message->net_recv;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_status_interfaces
size_t get_serialized_size_key_status_interfaces__msg__SystemStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _SystemStatus__ros_msg_type * ros_message = static_cast<const _SystemStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: stamp
  current_alignment += get_serialized_size_key_builtin_interfaces__msg__Time(
    &(ros_message->stamp), current_alignment);

  // Field name: host_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->host_name.size + 1);

  // Field name: cpu_percent
  {
    size_t item_size = sizeof(ros_message->cpu_percent);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: memory_percent
  {
    size_t item_size = sizeof(ros_message->memory_percent);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: memory_total
  {
    size_t item_size = sizeof(ros_message->memory_total);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: memory_available
  {
    size_t item_size = sizeof(ros_message->memory_available);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: net_sent
  {
    size_t item_size = sizeof(ros_message->net_sent);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  // Field name: net_recv
  {
    size_t item_size = sizeof(ros_message->net_recv);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_status_interfaces
size_t max_serialized_size_key_status_interfaces__msg__SystemStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;
  // Field name: stamp
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_builtin_interfaces__msg__Time(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: host_name
  {
    size_t array_size = 1;
    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Field name: cpu_percent
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: memory_percent
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: memory_total
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: memory_available
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Field name: net_sent
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Field name: net_recv
  {
    size_t array_size = 1;
    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = status_interfaces__msg__SystemStatus;
    is_plain =
      (
      offsetof(DataType, net_recv) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _SystemStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const status_interfaces__msg__SystemStatus * ros_message = static_cast<const status_interfaces__msg__SystemStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_status_interfaces__msg__SystemStatus(ros_message, cdr);
}

static bool _SystemStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  status_interfaces__msg__SystemStatus * ros_message = static_cast<status_interfaces__msg__SystemStatus *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_status_interfaces__msg__SystemStatus(cdr, ros_message);
}

static uint32_t _SystemStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_status_interfaces__msg__SystemStatus(
      untyped_ros_message, 0));
}

static size_t _SystemStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_status_interfaces__msg__SystemStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_SystemStatus = {
  "status_interfaces::msg",
  "SystemStatus",
  _SystemStatus__cdr_serialize,
  _SystemStatus__cdr_deserialize,
  _SystemStatus__get_serialized_size,
  _SystemStatus__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _SystemStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_SystemStatus,
  get_message_typesupport_handle_function,
  &status_interfaces__msg__SystemStatus__get_type_hash,
  &status_interfaces__msg__SystemStatus__get_type_description,
  &status_interfaces__msg__SystemStatus__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, status_interfaces, msg, SystemStatus)() {
  return &_SystemStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
