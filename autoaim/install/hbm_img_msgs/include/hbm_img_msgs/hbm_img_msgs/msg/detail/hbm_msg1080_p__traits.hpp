// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from hbm_img_msgs:msg/HbmMsg1080P.idl
// generated code does not contain a copyright notice

#ifndef HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__TRAITS_HPP_
#define HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "hbm_img_msgs/msg/detail/hbm_msg1080_p__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'time_stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace hbm_img_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const HbmMsg1080P & msg,
  std::ostream & out)
{
  out << "{";
  // member: index
  {
    out << "index: ";
    rosidl_generator_traits::value_to_yaml(msg.index, out);
    out << ", ";
  }

  // member: time_stamp
  {
    out << "time_stamp: ";
    to_flow_style_yaml(msg.time_stamp, out);
    out << ", ";
  }

  // member: height
  {
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: data_size
  {
    out << "data_size: ";
    rosidl_generator_traits::value_to_yaml(msg.data_size, out);
    out << ", ";
  }

  // member: step
  {
    out << "step: ";
    rosidl_generator_traits::value_to_yaml(msg.step, out);
    out << ", ";
  }

  // member: encoding
  {
    if (msg.encoding.size() == 0) {
      out << "encoding: []";
    } else {
      out << "encoding: [";
      size_t pending_items = msg.encoding.size();
      for (auto item : msg.encoding) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
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
  const HbmMsg1080P & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "index: ";
    rosidl_generator_traits::value_to_yaml(msg.index, out);
    out << "\n";
  }

  // member: time_stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_stamp:\n";
    to_block_style_yaml(msg.time_stamp, out, indentation + 2);
  }

  // member: height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "height: ";
    rosidl_generator_traits::value_to_yaml(msg.height, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: data_size
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data_size: ";
    rosidl_generator_traits::value_to_yaml(msg.data_size, out);
    out << "\n";
  }

  // member: step
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "step: ";
    rosidl_generator_traits::value_to_yaml(msg.step, out);
    out << "\n";
  }

  // member: encoding
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.encoding.size() == 0) {
      out << "encoding: []\n";
    } else {
      out << "encoding:\n";
      for (auto item : msg.encoding) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
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

inline std::string to_yaml(const HbmMsg1080P & msg, bool use_flow_style = false)
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

}  // namespace hbm_img_msgs

namespace rosidl_generator_traits
{

[[deprecated("use hbm_img_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const hbm_img_msgs::msg::HbmMsg1080P & msg,
  std::ostream & out, size_t indentation = 0)
{
  hbm_img_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use hbm_img_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const hbm_img_msgs::msg::HbmMsg1080P & msg)
{
  return hbm_img_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<hbm_img_msgs::msg::HbmMsg1080P>()
{
  return "hbm_img_msgs::msg::HbmMsg1080P";
}

template<>
inline const char * name<hbm_img_msgs::msg::HbmMsg1080P>()
{
  return "hbm_img_msgs/msg/HbmMsg1080P";
}

template<>
struct has_fixed_size<hbm_img_msgs::msg::HbmMsg1080P>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct has_bounded_size<hbm_img_msgs::msg::HbmMsg1080P>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value> {};

template<>
struct is_message<hbm_img_msgs::msg::HbmMsg1080P>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__TRAITS_HPP_
