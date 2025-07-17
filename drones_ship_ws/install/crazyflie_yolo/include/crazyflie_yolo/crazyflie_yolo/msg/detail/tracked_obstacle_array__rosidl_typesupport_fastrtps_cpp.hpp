// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from crazyflie_yolo:msg/TrackedObstacleArray.idl
// generated code does not contain a copyright notice

#ifndef CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "crazyflie_yolo/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "crazyflie_yolo/msg/detail/tracked_obstacle_array__struct.hpp"

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

#include "fastcdr/Cdr.h"

namespace crazyflie_yolo
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_yolo
cdr_serialize(
  const crazyflie_yolo::msg::TrackedObstacleArray & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_yolo
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  crazyflie_yolo::msg::TrackedObstacleArray & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_yolo
get_serialized_size(
  const crazyflie_yolo::msg::TrackedObstacleArray & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_yolo
max_serialized_size_TrackedObstacleArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_yolo
cdr_serialize_key(
  const crazyflie_yolo::msg::TrackedObstacleArray & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_yolo
get_serialized_size_key(
  const crazyflie_yolo::msg::TrackedObstacleArray & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_yolo
max_serialized_size_key_TrackedObstacleArray(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace crazyflie_yolo

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_crazyflie_yolo
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, crazyflie_yolo, msg, TrackedObstacleArray)();

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
