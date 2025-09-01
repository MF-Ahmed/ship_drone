// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from crazyflie_yolo:msg/TrackedObstacle.idl
// generated code does not contain a copyright notice
#ifndef CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "crazyflie_yolo/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "crazyflie_yolo/msg/detail/tracked_obstacle__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
bool cdr_serialize_crazyflie_yolo__msg__TrackedObstacle(
  const crazyflie_yolo__msg__TrackedObstacle * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
bool cdr_deserialize_crazyflie_yolo__msg__TrackedObstacle(
  eprosima::fastcdr::Cdr &,
  crazyflie_yolo__msg__TrackedObstacle * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
size_t get_serialized_size_crazyflie_yolo__msg__TrackedObstacle(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
size_t max_serialized_size_crazyflie_yolo__msg__TrackedObstacle(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
bool cdr_serialize_key_crazyflie_yolo__msg__TrackedObstacle(
  const crazyflie_yolo__msg__TrackedObstacle * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
size_t get_serialized_size_key_crazyflie_yolo__msg__TrackedObstacle(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
size_t max_serialized_size_key_crazyflie_yolo__msg__TrackedObstacle(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_crazyflie_yolo
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, crazyflie_yolo, msg, TrackedObstacle)();

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
