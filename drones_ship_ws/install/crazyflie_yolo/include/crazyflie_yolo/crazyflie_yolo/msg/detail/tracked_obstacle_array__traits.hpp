// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from crazyflie_yolo:msg/TrackedObstacleArray.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "crazyflie_yolo/msg/tracked_obstacle_array.hpp"


#ifndef CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__TRAITS_HPP_
#define CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "crazyflie_yolo/msg/detail/tracked_obstacle_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'obstacles'
#include "crazyflie_yolo/msg/detail/tracked_obstacle__traits.hpp"

namespace crazyflie_yolo
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrackedObstacleArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: obstacles
  {
    if (msg.obstacles.size() == 0) {
      out << "obstacles: []";
    } else {
      out << "obstacles: [";
      size_t pending_items = msg.obstacles.size();
      for (auto item : msg.obstacles) {
        to_flow_style_yaml(item, out);
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
  const TrackedObstacleArray & msg,
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

  // member: obstacles
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.obstacles.size() == 0) {
      out << "obstacles: []\n";
    } else {
      out << "obstacles:\n";
      for (auto item : msg.obstacles) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrackedObstacleArray & msg, bool use_flow_style = false)
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
  const crazyflie_yolo::msg::TrackedObstacleArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  crazyflie_yolo::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use crazyflie_yolo::msg::to_yaml() instead")]]
inline std::string to_yaml(const crazyflie_yolo::msg::TrackedObstacleArray & msg)
{
  return crazyflie_yolo::msg::to_yaml(msg);
}

template<>
inline const char * data_type<crazyflie_yolo::msg::TrackedObstacleArray>()
{
  return "crazyflie_yolo::msg::TrackedObstacleArray";
}

template<>
inline const char * name<crazyflie_yolo::msg::TrackedObstacleArray>()
{
  return "crazyflie_yolo/msg/TrackedObstacleArray";
}

template<>
struct has_fixed_size<crazyflie_yolo::msg::TrackedObstacleArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<crazyflie_yolo::msg::TrackedObstacleArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<crazyflie_yolo::msg::TrackedObstacleArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CRAZYFLIE_YOLO__MSG__DETAIL__TRACKED_OBSTACLE_ARRAY__TRAITS_HPP_
