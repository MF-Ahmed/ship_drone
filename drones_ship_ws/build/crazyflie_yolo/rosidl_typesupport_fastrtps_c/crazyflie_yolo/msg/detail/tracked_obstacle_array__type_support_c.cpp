// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from crazyflie_yolo:msg/TrackedObstacleArray.idl
// generated code does not contain a copyright notice
#include "crazyflie_yolo/msg/detail/tracked_obstacle_array__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <cstddef>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/serialization_helpers.hpp"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "crazyflie_yolo/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "crazyflie_yolo/msg/detail/tracked_obstacle_array__struct.h"
#include "crazyflie_yolo/msg/detail/tracked_obstacle_array__functions.h"
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

#include "crazyflie_yolo/msg/detail/tracked_obstacle__functions.h"  // obstacles
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions

bool cdr_serialize_crazyflie_yolo__msg__TrackedObstacle(
  const crazyflie_yolo__msg__TrackedObstacle * ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool cdr_deserialize_crazyflie_yolo__msg__TrackedObstacle(
  eprosima::fastcdr::Cdr & cdr,
  crazyflie_yolo__msg__TrackedObstacle * ros_message);

size_t get_serialized_size_crazyflie_yolo__msg__TrackedObstacle(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_crazyflie_yolo__msg__TrackedObstacle(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool cdr_serialize_key_crazyflie_yolo__msg__TrackedObstacle(
  const crazyflie_yolo__msg__TrackedObstacle * ros_message,
  eprosima::fastcdr::Cdr & cdr);

size_t get_serialized_size_key_crazyflie_yolo__msg__TrackedObstacle(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_key_crazyflie_yolo__msg__TrackedObstacle(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, crazyflie_yolo, msg, TrackedObstacle)();

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_crazyflie_yolo
bool cdr_serialize_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_crazyflie_yolo
bool cdr_deserialize_std_msgs__msg__Header(
  eprosima::fastcdr::Cdr & cdr,
  std_msgs__msg__Header * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_crazyflie_yolo
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_crazyflie_yolo
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_crazyflie_yolo
bool cdr_serialize_key_std_msgs__msg__Header(
  const std_msgs__msg__Header * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_crazyflie_yolo
size_t get_serialized_size_key_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_crazyflie_yolo
size_t max_serialized_size_key_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_crazyflie_yolo
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _TrackedObstacleArray__ros_msg_type = crazyflie_yolo__msg__TrackedObstacleArray;


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
bool cdr_serialize_crazyflie_yolo__msg__TrackedObstacleArray(
  const crazyflie_yolo__msg__TrackedObstacleArray * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: obstacles
  {
    size_t size = ros_message->obstacles.size;
    auto array_ptr = ros_message->obstacles.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_crazyflie_yolo__msg__TrackedObstacle(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
bool cdr_deserialize_crazyflie_yolo__msg__TrackedObstacleArray(
  eprosima::fastcdr::Cdr & cdr,
  crazyflie_yolo__msg__TrackedObstacleArray * ros_message)
{
  // Field name: header
  {
    cdr_deserialize_std_msgs__msg__Header(cdr, &ros_message->header);
  }

  // Field name: obstacles
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->obstacles.data) {
      crazyflie_yolo__msg__TrackedObstacle__Sequence__fini(&ros_message->obstacles);
    }
    if (!crazyflie_yolo__msg__TrackedObstacle__Sequence__init(&ros_message->obstacles, size)) {
      fprintf(stderr, "failed to create array for field 'obstacles'");
      return false;
    }
    auto array_ptr = ros_message->obstacles.data;
    for (size_t i = 0; i < size; ++i) {
      cdr_deserialize_crazyflie_yolo__msg__TrackedObstacle(cdr, &array_ptr[i]);
    }
  }

  return true;
}  // NOLINT(readability/fn_size)


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
size_t get_serialized_size_crazyflie_yolo__msg__TrackedObstacleArray(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TrackedObstacleArray__ros_msg_type * ros_message = static_cast<const _TrackedObstacleArray__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: obstacles
  {
    size_t array_size = ros_message->obstacles.size;
    auto array_ptr = ros_message->obstacles.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_crazyflie_yolo__msg__TrackedObstacle(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}


ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
size_t max_serialized_size_crazyflie_yolo__msg__TrackedObstacleArray(
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

  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: obstacles
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_crazyflie_yolo__msg__TrackedObstacle(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }


  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = crazyflie_yolo__msg__TrackedObstacleArray;
    is_plain =
      (
      offsetof(DataType, obstacles) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
bool cdr_serialize_key_crazyflie_yolo__msg__TrackedObstacleArray(
  const crazyflie_yolo__msg__TrackedObstacleArray * ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Field name: header
  {
    cdr_serialize_key_std_msgs__msg__Header(
      &ros_message->header, cdr);
  }

  // Field name: obstacles
  {
    size_t size = ros_message->obstacles.size;
    auto array_ptr = ros_message->obstacles.data;
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      cdr_serialize_key_crazyflie_yolo__msg__TrackedObstacle(
        &array_ptr[i], cdr);
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
size_t get_serialized_size_key_crazyflie_yolo__msg__TrackedObstacleArray(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TrackedObstacleArray__ros_msg_type * ros_message = static_cast<const _TrackedObstacleArray__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;

  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Field name: header
  current_alignment += get_serialized_size_key_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);

  // Field name: obstacles
  {
    size_t array_size = ros_message->obstacles.size;
    auto array_ptr = ros_message->obstacles.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_key_crazyflie_yolo__msg__TrackedObstacle(
        &array_ptr[index], current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
size_t max_serialized_size_key_crazyflie_yolo__msg__TrackedObstacleArray(
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
  // Field name: header
  {
    size_t array_size = 1;
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Field name: obstacles
  {
    size_t array_size = 0;
    full_bounded = false;
    is_plain = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_key_crazyflie_yolo__msg__TrackedObstacle(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = crazyflie_yolo__msg__TrackedObstacleArray;
    is_plain =
      (
      offsetof(DataType, obstacles) +
      last_member_size
      ) == ret_val;
  }
  return ret_val;
}


static bool _TrackedObstacleArray__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const crazyflie_yolo__msg__TrackedObstacleArray * ros_message = static_cast<const crazyflie_yolo__msg__TrackedObstacleArray *>(untyped_ros_message);
  (void)ros_message;
  return cdr_serialize_crazyflie_yolo__msg__TrackedObstacleArray(ros_message, cdr);
}

static bool _TrackedObstacleArray__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  crazyflie_yolo__msg__TrackedObstacleArray * ros_message = static_cast<crazyflie_yolo__msg__TrackedObstacleArray *>(untyped_ros_message);
  (void)ros_message;
  return cdr_deserialize_crazyflie_yolo__msg__TrackedObstacleArray(cdr, ros_message);
}

static uint32_t _TrackedObstacleArray__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_crazyflie_yolo__msg__TrackedObstacleArray(
      untyped_ros_message, 0));
}

static size_t _TrackedObstacleArray__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_crazyflie_yolo__msg__TrackedObstacleArray(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TrackedObstacleArray = {
  "crazyflie_yolo::msg",
  "TrackedObstacleArray",
  _TrackedObstacleArray__cdr_serialize,
  _TrackedObstacleArray__cdr_deserialize,
  _TrackedObstacleArray__get_serialized_size,
  _TrackedObstacleArray__max_serialized_size,
  nullptr
};

static rosidl_message_type_support_t _TrackedObstacleArray__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TrackedObstacleArray,
  get_message_typesupport_handle_function,
  &crazyflie_yolo__msg__TrackedObstacleArray__get_type_hash,
  &crazyflie_yolo__msg__TrackedObstacleArray__get_type_description,
  &crazyflie_yolo__msg__TrackedObstacleArray__get_type_description_sources,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, crazyflie_yolo, msg, TrackedObstacleArray)() {
  return &_TrackedObstacleArray__type_support;
}

#if defined(__cplusplus)
}
#endif
