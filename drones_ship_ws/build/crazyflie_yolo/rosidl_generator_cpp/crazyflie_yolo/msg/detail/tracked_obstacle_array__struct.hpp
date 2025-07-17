// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crazyflie_yolo:msg/TrackedObstacleArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"


#ifndef CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__STRUCT_HPP_
#define CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'obstacles'
#include "crazyflie_yolo/msg/detail/tracked_obstacle__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__crazyflie_yolo__msg__TrackedObstacleArray __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_yolo__msg__TrackedObstacleArray __declspec(deprecated)
#endif

namespace crazyflie_yolo
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrackedObstacleArray_
{
  using Type = TrackedObstacleArray_<ContainerAllocator>;

  explicit TrackedObstacleArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit TrackedObstacleArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _obstacles_type =
    std::vector<crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator>>>;
  _obstacles_type obstacles;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__obstacles(
    const std::vector<crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator>>> & _arg)
  {
    this->obstacles = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crazyflie_yolo::msg::TrackedObstacleArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_yolo::msg::TrackedObstacleArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_yolo::msg::TrackedObstacleArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_yolo::msg::TrackedObstacleArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_yolo::msg::TrackedObstacleArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_yolo::msg::TrackedObstacleArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_yolo::msg::TrackedObstacleArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_yolo::msg::TrackedObstacleArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_yolo::msg::TrackedObstacleArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_yolo::msg::TrackedObstacleArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_yolo__msg__TrackedObstacleArray
    std::shared_ptr<crazyflie_yolo::msg::TrackedObstacleArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_yolo__msg__TrackedObstacleArray
    std::shared_ptr<crazyflie_yolo::msg::TrackedObstacleArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackedObstacleArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->obstacles != other.obstacles) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackedObstacleArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackedObstacleArray_

// alias to use template instance with default allocator
using TrackedObstacleArray =
  crazyflie_yolo::msg::TrackedObstacleArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace crazyflie_yolo

#endif  // CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__STRUCT_HPP_
