// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from crazyflie_yolo:msg/TrackedObstacle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "crazyflie_yolo/msg/tracked_obstacle.hpp"


#ifndef CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__STRUCT_HPP_
#define CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__STRUCT_HPP_

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
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__crazyflie_yolo__msg__TrackedObstacle __attribute__((deprecated))
#else
# define DEPRECATED__crazyflie_yolo__msg__TrackedObstacle __declspec(deprecated)
#endif

namespace crazyflie_yolo
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrackedObstacle_
{
  using Type = TrackedObstacle_<ContainerAllocator>;

  explicit TrackedObstacle_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    position(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      std::fill<typename std::array<double, 9>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
    }
  }

  explicit TrackedObstacle_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    position(_alloc, _init),
    covariance(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      std::fill<typename std::array<double, 9>::iterator, double>(this->covariance.begin(), this->covariance.end(), 0.0);
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _id_type =
    int32_t;
  _id_type id;
  using _position_type =
    geometry_msgs::msg::Point_<ContainerAllocator>;
  _position_type position;
  using _covariance_type =
    std::array<double, 9>;
  _covariance_type covariance;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__position(
    const geometry_msgs::msg::Point_<ContainerAllocator> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__covariance(
    const std::array<double, 9> & _arg)
  {
    this->covariance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator> *;
  using ConstRawPtr =
    const crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__crazyflie_yolo__msg__TrackedObstacle
    std::shared_ptr<crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__crazyflie_yolo__msg__TrackedObstacle
    std::shared_ptr<crazyflie_yolo::msg::TrackedObstacle_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackedObstacle_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->covariance != other.covariance) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackedObstacle_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackedObstacle_

// alias to use template instance with default allocator
using TrackedObstacle =
  crazyflie_yolo::msg::TrackedObstacle_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace crazyflie_yolo

#endif  // CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__STRUCT_HPP_
