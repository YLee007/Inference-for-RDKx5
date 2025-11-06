// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from vision_interfaces:msg/AutoAim.idl
// generated code does not contain a copyright notice

#ifndef VISION_INTERFACES__MSG__DETAIL__AUTO_AIM__TRAITS_HPP_
#define VISION_INTERFACES__MSG__DETAIL__AUTO_AIM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "vision_interfaces/msg/detail/auto_aim__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace vision_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const AutoAim & msg,
  std::ostream & out)
{
  out << "{";
  // member: aim_yaw
  {
    out << "aim_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.aim_yaw, out);
    out << ", ";
  }

  // member: aim_pitch
  {
    out << "aim_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.aim_pitch, out);
    out << ", ";
  }

  // member: fire
  {
    out << "fire: ";
    rosidl_generator_traits::value_to_yaml(msg.fire, out);
    out << ", ";
  }

  // member: tracking
  {
    out << "tracking: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AutoAim & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: aim_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aim_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.aim_yaw, out);
    out << "\n";
  }

  // member: aim_pitch
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "aim_pitch: ";
    rosidl_generator_traits::value_to_yaml(msg.aim_pitch, out);
    out << "\n";
  }

  // member: fire
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fire: ";
    rosidl_generator_traits::value_to_yaml(msg.fire, out);
    out << "\n";
  }

  // member: tracking
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tracking: ";
    rosidl_generator_traits::value_to_yaml(msg.tracking, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AutoAim & msg, bool use_flow_style = false)
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
  const vision_interfaces::msg::AutoAim & msg,
  std::ostream & out, size_t indentation = 0)
{
  vision_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use vision_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const vision_interfaces::msg::AutoAim & msg)
{
  return vision_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<vision_interfaces::msg::AutoAim>()
{
  return "vision_interfaces::msg::AutoAim";
}

template<>
inline const char * name<vision_interfaces::msg::AutoAim>()
{
  return "vision_interfaces/msg/AutoAim";
}

template<>
struct has_fixed_size<vision_interfaces::msg::AutoAim>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<vision_interfaces::msg::AutoAim>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<vision_interfaces::msg::AutoAim>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // VISION_INTERFACES__MSG__DETAIL__AUTO_AIM__TRAITS_HPP_
