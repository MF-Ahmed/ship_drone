// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from crazyflie_yolo:msg/TrackedObstacleArray.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "crazyflie_yolo/msg/detail/tracked_obstacle_array__functions.h"
#include "crazyflie_yolo/msg/detail/tracked_obstacle_array__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace crazyflie_yolo
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TrackedObstacleArray_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) crazyflie_yolo::msg::TrackedObstacleArray(_init);
}

void TrackedObstacleArray_fini_function(void * message_memory)
{
  auto typed_message = static_cast<crazyflie_yolo::msg::TrackedObstacleArray *>(message_memory);
  typed_message->~TrackedObstacleArray();
}

size_t size_function__TrackedObstacleArray__obstacles(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<crazyflie_yolo::msg::TrackedObstacle> *>(untyped_member);
  return member->size();
}

const void * get_const_function__TrackedObstacleArray__obstacles(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<crazyflie_yolo::msg::TrackedObstacle> *>(untyped_member);
  return &member[index];
}

void * get_function__TrackedObstacleArray__obstacles(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<crazyflie_yolo::msg::TrackedObstacle> *>(untyped_member);
  return &member[index];
}

void fetch_function__TrackedObstacleArray__obstacles(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const crazyflie_yolo::msg::TrackedObstacle *>(
    get_const_function__TrackedObstacleArray__obstacles(untyped_member, index));
  auto & value = *reinterpret_cast<crazyflie_yolo::msg::TrackedObstacle *>(untyped_value);
  value = item;
}

void assign_function__TrackedObstacleArray__obstacles(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<crazyflie_yolo::msg::TrackedObstacle *>(
    get_function__TrackedObstacleArray__obstacles(untyped_member, index));
  const auto & value = *reinterpret_cast<const crazyflie_yolo::msg::TrackedObstacle *>(untyped_value);
  item = value;
}

void resize_function__TrackedObstacleArray__obstacles(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<crazyflie_yolo::msg::TrackedObstacle> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TrackedObstacleArray_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_yolo::msg::TrackedObstacleArray, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "obstacles",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<crazyflie_yolo::msg::TrackedObstacle>(),  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(crazyflie_yolo::msg::TrackedObstacleArray, obstacles),  // bytes offset in struct
    nullptr,  // default value
    size_function__TrackedObstacleArray__obstacles,  // size() function pointer
    get_const_function__TrackedObstacleArray__obstacles,  // get_const(index) function pointer
    get_function__TrackedObstacleArray__obstacles,  // get(index) function pointer
    fetch_function__TrackedObstacleArray__obstacles,  // fetch(index, &value) function pointer
    assign_function__TrackedObstacleArray__obstacles,  // assign(index, value) function pointer
    resize_function__TrackedObstacleArray__obstacles  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TrackedObstacleArray_message_members = {
  "crazyflie_yolo::msg",  // message namespace
  "TrackedObstacleArray",  // message name
  2,  // number of fields
  sizeof(crazyflie_yolo::msg::TrackedObstacleArray),
  false,  // has_any_key_member_
  TrackedObstacleArray_message_member_array,  // message members
  TrackedObstacleArray_init_function,  // function to initialize message memory (memory has to be allocated)
  TrackedObstacleArray_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TrackedObstacleArray_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TrackedObstacleArray_message_members,
  get_message_typesupport_handle_function,
  &crazyflie_yolo__msg__TrackedObstacleArray__get_type_hash,
  &crazyflie_yolo__msg__TrackedObstacleArray__get_type_description,
  &crazyflie_yolo__msg__TrackedObstacleArray__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace crazyflie_yolo


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<crazyflie_yolo::msg::TrackedObstacleArray>()
{
  return &::crazyflie_yolo::msg::rosidl_typesupport_introspection_cpp::TrackedObstacleArray_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, crazyflie_yolo, msg, TrackedObstacleArray)() {
  return &::crazyflie_yolo::msg::rosidl_typesupport_introspection_cpp::TrackedObstacleArray_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
