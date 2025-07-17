// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from crazyflie_yolo:msg/TrackedObstacleArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "crazyflie_yolo/msg/tracked_obstacle_array.h"


#ifndef CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__STRUCT_H_
#define CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__STRUCT_H_

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
// Member 'obstacles'
#include "crazyflie_yolo/msg/detail/tracked_obstacle__struct.h"

/// Struct defined in msg/TrackedObstacleArray in the package crazyflie_yolo.
/**
  * TrackedObstacleArray.msg
 */
typedef struct crazyflie_yolo__msg__TrackedObstacleArray
{
  std_msgs__msg__Header header;
  crazyflie_yolo__msg__TrackedObstacle__Sequence obstacles;
} crazyflie_yolo__msg__TrackedObstacleArray;

// Struct for a sequence of crazyflie_yolo__msg__TrackedObstacleArray.
typedef struct crazyflie_yolo__msg__TrackedObstacleArray__Sequence
{
  crazyflie_yolo__msg__TrackedObstacleArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} crazyflie_yolo__msg__TrackedObstacleArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__STRUCT_H_
