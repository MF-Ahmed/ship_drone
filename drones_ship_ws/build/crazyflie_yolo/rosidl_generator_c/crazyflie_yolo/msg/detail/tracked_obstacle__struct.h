// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_yolo:msg/TrackedObstacle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "crazyflie_yolo/msg/tracked_obstacle.h"


#ifndef CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__STRUCT_H_
#define CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/TrackedObstacle in the package crazyflie_yolo.
/**
  * TrackedObstacle.msg
 */
typedef struct crazyflie_yolo__msg__TrackedObstacle
{
  /// For timestamp and frame_id
  std_msgs__msg__Header header;
  /// Track ID
  int32_t id;
  geometry_msgs__msg__Point position;
  /// Flattened 3x3 covariance matrix
  double covariance[9];
} crazyflie_yolo__msg__TrackedObstacle;

// Struct for a sequence of crazyflie_yolo__msg__TrackedObstacle.
typedef struct crazyflie_yolo__msg__TrackedObstacle__Sequence
{
  crazyflie_yolo__msg__TrackedObstacle * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_yolo__msg__TrackedObstacle__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__STRUCT_H_
