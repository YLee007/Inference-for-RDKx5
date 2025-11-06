// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from hbm_img_msgs:msg/HbmMsg1080P.idl
// generated code does not contain a copyright notice

#ifndef HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "hbm_img_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "hbm_img_msgs/msg/detail/hbm_msg1080_p__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace hbm_img_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hbm_img_msgs
cdr_serialize(
  const hbm_img_msgs::msg::HbmMsg1080P & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hbm_img_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hbm_img_msgs::msg::HbmMsg1080P & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hbm_img_msgs
get_serialized_size(
  const hbm_img_msgs::msg::HbmMsg1080P & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hbm_img_msgs
max_serialized_size_HbmMsg1080P(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace hbm_img_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hbm_img_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hbm_img_msgs, msg, HbmMsg1080P)();

#ifdef __cplusplus
}
#endif

#endif  // HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
