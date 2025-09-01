// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_yolo:msg/TrackedObstacle.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "crazyflie_yolo/msg/tracked_obstacle.hpp"


#ifndef CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__TRAITS_HPP_
#define CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_yolo/msg/detail/tracked_obstacle__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'position'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace crazyflie_yolo
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrackedObstacle & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: position
  {
    out << "position: ";
    to_flow_style_yaml(msg.position, out);
    out << ", ";
  }

  // member: covariance
  {
    if (msg.covariance.size() == 0) {
      out << "covariance: []";
    } else {
      out << "covariance: [";
      size_t pending_items = msg.covariance.size();
      for (auto item : msg.covariance) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrackedObstacle & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: position
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "position:\n";
    to_block_style_yaml(msg.position, out, indentation + 2);
  }

  // member: covariance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.covariance.size() == 0) {
      out << "covariance: []\n";
    } else {
      out << "covariance:\n";
      for (auto item : msg.covariance) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrackedObstacle & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace crazyflie_yolo

namespace rosidl_generator_traits
{

[[deprecated("use crazyflie_yolo::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const crazyflie_yolo::msg::TrackedObstacle & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_yolo::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_yolo::msg::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_yolo::msg::TrackedObstacle & msg)
{
  return crazyflie_yolo::msg::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_yolo::msg::TrackedObstacle>()
{
  return "crazyflie_yolo::msg::TrackedObstacle";
}

template<>
inline const char * name<crazyflie_yolo::msg::TrackedObstacle>()
{
  return "crazyflie_yolo/msg/TrackedObstacle";
}

template<>
struct has_fixed_size<crazyflie_yolo::msg::TrackedObstacle>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<crazyflie_yolo::msg::TrackedObstacle>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<crazyflie_yolo::msg::TrackedObstacle>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE__TRAITS_HPP_
