// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from crazyflie_yolo:msg/TrackedObstacle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "crazyflie_yolo/msg/tracked_obstacle.hpp"


#ifndef CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__BUILDER_HPP_
#define CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "crazyflie_yolo/msg/detail/tracked_obstacle__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace crazyflie_yolo
{

namespace msg
{

namespace builder
{

class Init_TrackedObstacle_covariance
{
public:
  explicit Init_TrackedObstacle_covariance(::crazyflie_yolo::msg::TrackedObstacle & msg)
  : msg_(msg)
  {}
  ::crazyflie_yolo::msg::TrackedObstacle covariance(::crazyflie_yolo::msg::TrackedObstacle::_covariance_type arg)
  {
    msg_.covariance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::crazyflie_yolo::msg::TrackedObstacle msg_;
};

class Init_TrackedObstacle_position
{
public:
  explicit Init_TrackedObstacle_position(::crazyflie_yolo::msg::TrackedObstacle & msg)
  : msg_(msg)
  {}
  Init_TrackedObstacle_covariance position(::crazyflie_yolo::msg::TrackedObstacle::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_TrackedObstacle_covariance(msg_);
  }

private:
  ::crazyflie_yolo::msg::TrackedObstacle msg_;
};

class Init_TrackedObstacle_id
{
public:
  explicit Init_TrackedObstacle_id(::crazyflie_yolo::msg::TrackedObstacle & msg)
  : msg_(msg)
  {}
  Init_TrackedObstacle_position id(::crazyflie_yolo::msg::TrackedObstacle::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_TrackedObstacle_position(msg_);
  }

private:
  ::crazyflie_yolo::msg::TrackedObstacle msg_;
};

class Init_TrackedObstacle_header
{
public:
  Init_TrackedObstacle_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackedObstacle_id header(::crazyflie_yolo::msg::TrackedObstacle::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TrackedObstacle_id(msg_);
  }

private:
  ::crazyflie_yolo::msg::TrackedObstacle msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::crazyflie_yolo::msg::TrackedObstacle>()
{
  return crazyflie_yolo::msg::builder::Init_TrackedObstacle_header();
}

}  // namespace crazyflie_yolo

#endif  // CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__BUILDER_HPP_
