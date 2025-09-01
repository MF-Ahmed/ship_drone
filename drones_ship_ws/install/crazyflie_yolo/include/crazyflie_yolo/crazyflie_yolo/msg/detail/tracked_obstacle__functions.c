// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from crazyflie_yolo:msg/TrackedObstacle.idl
// generated code does not contain a copyright notice
#include "crazyflie_yolo/msg/detail/tracked_obstacle__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
crazyflie_yolo__msg__TrackedObstacle__init(crazyflie_yolo__msg__TrackedObstacle * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    crazyflie_yolo__msg__TrackedObstacle__fini(msg);
    return false;
  }
  // id
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    crazyflie_yolo__msg__TrackedObstacle__fini(msg);
    return false;
  }
  // covariance
  return true;
}

void
crazyflie_yolo__msg__TrackedObstacle__fini(crazyflie_yolo__msg__TrackedObstacle * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // id
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // covariance
}

bool
crazyflie_yolo__msg__TrackedObstacle__are_equal(const crazyflie_yolo__msg__TrackedObstacle * lhs, const crazyflie_yolo__msg__TrackedObstacle * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // covariance
  for (size_t i = 0; i < 9; ++i) {
    if (lhs->covariance[i] != rhs->covariance[i]) {
      return false;
    }
  }
  return true;
}

bool
crazyflie_yolo__msg__TrackedObstacle__copy(
  const crazyflie_yolo__msg__TrackedObstacle * input,
  crazyflie_yolo__msg__TrackedObstacle * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // id
  output->id = input->id;
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // covariance
  for (size_t i = 0; i < 9; ++i) {
    output->covariance[i] = input->covariance[i];
  }
  return true;
}

crazyflie_yolo__msg__TrackedObstacle *
crazyflie_yolo__msg__TrackedObstacle__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_yolo__msg__TrackedObstacle * msg = (crazyflie_yolo__msg__TrackedObstacle *)allocator.allocate(sizeof(crazyflie_yolo__msg__TrackedObstacle), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(crazyflie_yolo__msg__TrackedObstacle));
  bool success = crazyflie_yolo__msg__TrackedObstacle__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
crazyflie_yolo__msg__TrackedObstacle__destroy(crazyflie_yolo__msg__TrackedObstacle * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    crazyflie_yolo__msg__TrackedObstacle__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
crazyflie_yolo__msg__TrackedObstacle__Sequence__init(crazyflie_yolo__msg__TrackedObstacle__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_yolo__msg__TrackedObstacle * data = NULL;

  if (size) {
    data = (crazyflie_yolo__msg__TrackedObstacle *)allocator.zero_allocate(size, sizeof(crazyflie_yolo__msg__TrackedObstacle), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = crazyflie_yolo__msg__TrackedObstacle__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        crazyflie_yolo__msg__TrackedObstacle__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
crazyflie_yolo__msg__TrackedObstacle__Sequence__fini(crazyflie_yolo__msg__TrackedObstacle__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      crazyflie_yolo__msg__TrackedObstacle__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

crazyflie_yolo__msg__TrackedObstacle__Sequence *
crazyflie_yolo__msg__TrackedObstacle__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  crazyflie_yolo__msg__TrackedObstacle__Sequence * array = (crazyflie_yolo__msg__TrackedObstacle__Sequence *)allocator.allocate(sizeof(crazyflie_yolo__msg__TrackedObstacle__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = crazyflie_yolo__msg__TrackedObstacle__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
crazyflie_yolo__msg__TrackedObstacle__Sequence__destroy(crazyflie_yolo__msg__TrackedObstacle__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    crazyflie_yolo__msg__TrackedObstacle__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
crazyflie_yolo__msg__TrackedObstacle__Sequence__are_equal(const crazyflie_yolo__msg__TrackedObstacle__Sequence * lhs, const crazyflie_yolo__msg__TrackedObstacle__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!crazyflie_yolo__msg__TrackedObstacle__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
crazyflie_yolo__msg__TrackedObstacle__Sequence__copy(
  const crazyflie_yolo__msg__TrackedObstacle__Sequence * input,
  crazyflie_yolo__msg__TrackedObstacle__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(crazyflie_yolo__msg__TrackedObstacle);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    crazyflie_yolo__msg__TrackedObstacle * data =
      (crazyflie_yolo__msg__TrackedObstacle *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!crazyflie_yolo__msg__TrackedObstacle__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          crazyflie_yolo__msg__TrackedObstacle__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!crazyflie_yolo__msg__TrackedObstacle__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
