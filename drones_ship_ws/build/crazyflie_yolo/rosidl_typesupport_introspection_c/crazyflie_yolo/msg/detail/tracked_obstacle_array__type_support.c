// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from crazyflie_yolo:msg/TrackedObstacleArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "crazyflie_yolo/msg/detail/tracked_obstacle_array__rosidl_typesupport_introspection_c.h"
#include "crazyflie_yolo/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "crazyflie_yolo/msg/detail/tracked_obstacle_array__functions.h"
#include "crazyflie_yolo/msg/detail/tracked_obstacle_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `obstacles`
#include "crazyflie_yolo/msg/tracked_obstacle.h"
// Member `obstacles`
#include "crazyflie_yolo/msg/detail/tracked_obstacle__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__TrackedObstacleArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  crazyflie_yolo__msg__TrackedObstacleArray__init(message_memory);
}

void crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__TrackedObstacleArray_fini_function(void * message_memory)
{
  crazyflie_yolo__msg__TrackedObstacleArray__fini(message_memory);
}

size_t crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__size_function__TrackedObstacleArray__obstacles(
  const void * untyped_member)
{
  const crazyflie_yolo__msg__TrackedObstacle__Sequence * member =
    (const crazyflie_yolo__msg__TrackedObstacle__Sequence *)(untyped_member);
  return member->size;
}

const void * crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__get_const_function__TrackedObstacleArray__obstacles(
  const void * untyped_member, size_t index)
{
  const crazyflie_yolo__msg__TrackedObstacle__Sequence * member =
    (const crazyflie_yolo__msg__TrackedObstacle__Sequence *)(untyped_member);
  return &member->data[index];
}

void * crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__get_function__TrackedObstacleArray__obstacles(
  void * untyped_member, size_t index)
{
  crazyflie_yolo__msg__TrackedObstacle__Sequence * member =
    (crazyflie_yolo__msg__TrackedObstacle__Sequence *)(untyped_member);
  return &member->data[index];
}

void crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__fetch_function__TrackedObstacleArray__obstacles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const crazyflie_yolo__msg__TrackedObstacle * item =
    ((const crazyflie_yolo__msg__TrackedObstacle *)
    crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__get_const_function__TrackedObstacleArray__obstacles(untyped_member, index));
  crazyflie_yolo__msg__TrackedObstacle * value =
    (crazyflie_yolo__msg__TrackedObstacle *)(untyped_value);
  *value = *item;
}

void crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__assign_function__TrackedObstacleArray__obstacles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  crazyflie_yolo__msg__TrackedObstacle * item =
    ((crazyflie_yolo__msg__TrackedObstacle *)
    crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__get_function__TrackedObstacleArray__obstacles(untyped_member, index));
  const crazyflie_yolo__msg__TrackedObstacle * value =
    (const crazyflie_yolo__msg__TrackedObstacle *)(untyped_value);
  *item = *value;
}

bool crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__resize_function__TrackedObstacleArray__obstacles(
  void * untyped_member, size_t size)
{
  crazyflie_yolo__msg__TrackedObstacle__Sequence * member =
    (crazyflie_yolo__msg__TrackedObstacle__Sequence *)(untyped_member);
  crazyflie_yolo__msg__TrackedObstacle__Sequence__fini(member);
  return crazyflie_yolo__msg__TrackedObstacle__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__TrackedObstacleArray_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_yolo__msg__TrackedObstacleArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "obstacles",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_yolo__msg__TrackedObstacleArray, obstacles),  // bytes offset in struct
    NULL,  // default value
    crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__size_function__TrackedObstacleArray__obstacles,  // size() function pointer
    crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__get_const_function__TrackedObstacleArray__obstacles,  // get_const(index) function pointer
    crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__get_function__TrackedObstacleArray__obstacles,  // get(index) function pointer
    crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__fetch_function__TrackedObstacleArray__obstacles,  // fetch(index, &value) function pointer
    crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__assign_function__TrackedObstacleArray__obstacles,  // assign(index, value) function pointer
    crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__resize_function__TrackedObstacleArray__obstacles  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__TrackedObstacleArray_message_members = {
  "crazyflie_yolo__msg",  // message namespace
  "TrackedObstacleArray",  // message name
  2,  // number of fields
  sizeof(crazyflie_yolo__msg__TrackedObstacleArray),
  false,  // has_any_key_member_
  crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__TrackedObstacleArray_message_member_array,  // message members
  crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__TrackedObstacleArray_init_function,  // function to initialize message memory (memory has to be allocated)
  crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__TrackedObstacleArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__TrackedObstacleArray_message_type_support_handle = {
  0,
  &crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__TrackedObstacleArray_message_members,
  get_message_typesupport_handle_function,
  &crazyflie_yolo__msg__TrackedObstacleArray__get_type_hash,
  &crazyflie_yolo__msg__TrackedObstacleArray__get_type_description,
  &crazyflie_yolo__msg__TrackedObstacleArray__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_crazyflie_yolo
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_yolo, msg, TrackedObstacleArray)() {
  crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__TrackedObstacleArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__TrackedObstacleArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, crazyflie_yolo, msg, TrackedObstacle)();
  if (!crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__TrackedObstacleArray_message_type_support_handle.typesupport_identifier) {
    crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__TrackedObstacleArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &crazyflie_yolo__msg__TrackedObstacleArray__rosidl_typesupport_introspection_c__TrackedObstacleArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
