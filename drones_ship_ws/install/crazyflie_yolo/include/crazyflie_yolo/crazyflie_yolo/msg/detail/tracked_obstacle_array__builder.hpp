// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_yolo:msg/TrackedObstacleArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"


#ifndef CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__BUILDER_HPP_
#define CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_yolo/msg/detail/tracked_obstacle_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_yolo
{

namespace msg
{

namespace builder
{

class Init_TrackedObstacleArray_obstacles
{
public:
  explicit Init_TrackedObstacleArray_obstacles(::crazyflie_yolo::msg::TrackedObstacleArray & msg)
  : msg_(msg)
  {}
  ::crazyflie_yolo::msg::TrackedObstacleArray obstacles(::crazyflie_yolo::msg::TrackedObstacleArray::_obstacles_type arg)
  {
    msg_.obstacles = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_yolo::msg::TrackedObstacleArray msg_;
};

class Init_TrackedObstacleArray_header
{
public:
  Init_TrackedObstacleArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackedObstacleArray_obstacles header(::crazyflie_yolo::msg::TrackedObstacleArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TrackedObstacleArray_obstacles(msg_);
  }

private:
  ::crazyflie_yolo::msg::TrackedObstacleArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_yolo::msg::TrackedObstacleArray>()
{
  return crazyflie_yolo::msg::builder::Init_TrackedObstacleArray_header();
}

}  // namespace crazyflie_yolo

#endif  // CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__BUILDER_HPP_
