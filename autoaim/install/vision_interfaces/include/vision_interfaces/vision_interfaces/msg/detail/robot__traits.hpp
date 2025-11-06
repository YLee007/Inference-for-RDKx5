// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vision_interfaces:msg/Robot.idl
// generated code does not contain a copyright notice

#ifndef VISION_INTERFACES__MSG__DETAIL__ROBOT__TRAITS_HPP_
#define VISION_INTERFACES__MSG__DETAIL__ROBOT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vision_interfaces/msg/detail/robot__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vision_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Robot & msg,
  std::ostream & out)
{
  out << "{";
  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: foe_color
  {
    out << "foe_color: ";
    rosidl_generator_traits::value_to_yaml(msg.foe_color, out);
    out << ", ";
  }

  // member: self_yaw
  {
    out << "self_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.self_yaw, out);
    out << ", ";
  }

  // member: self_pitch
  {
    out << "self_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.self_pitch, out);
    out << ", ";
  }

  // member: muzzle_speed
  {
    out << "muzzle_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.muzzle_speed, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Robot & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: foe_color
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "foe_color: ";
    rosidl_generator_traits::value_to_yaml(msg.foe_color, out);
    out << "\n";
  }

  // member: self_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "self_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.self_yaw, out);
    out << "\n";
  }

  // member: self_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "self_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.self_pitch, out);
    out << "\n";
  }

  // member: muzzle_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "muzzle_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.muzzle_speed, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Robot & msg, bool use_flow_style = false)
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

}  // namespace vision_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use vision_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const vision_interfaces::msg::Robot & msg,
  std::ostream & out, size_t indentation = 0)
{
  vision_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vision_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const vision_interfaces::msg::Robot & msg)
{
  return vision_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vision_interfaces::msg::Robot>()
{
  return "vision_interfaces::msg::Robot";
}

template<>
inline const char * name<vision_interfaces::msg::Robot>()
{
  return "vision_interfaces/msg/Robot";
}

template<>
struct has_fixed_size<vision_interfaces::msg::Robot>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vision_interfaces::msg::Robot>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vision_interfaces::msg::Robot>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VISION_INTERFACES__MSG__DETAIL__ROBOT__TRAITS_HPP_
