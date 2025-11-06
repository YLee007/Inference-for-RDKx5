// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from vision_interfaces:msg/AutoAim.idl
// generated code does not contain a copyright notice
#include "vision_interfaces/msg/detail/auto_aim__rosidl_typesupport_fastrtps_cpp.hpp"
#include "vision_interfaces/msg/detail/auto_aim__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace vision_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vision_interfaces
cdr_serialize(
  const vision_interfaces::msg::AutoAim & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: aim_yaw
  cdr << ros_message.aim_yaw;
  // Member: aim_pitch
  cdr << ros_message.aim_pitch;
  // Member: fire
  cdr << ros_message.fire;
  // Member: tracking
  cdr << ros_message.tracking;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vision_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  vision_interfaces::msg::AutoAim & ros_message)
{
  // Member: aim_yaw
  cdr >> ros_message.aim_yaw;

  // Member: aim_pitch
  cdr >> ros_message.aim_pitch;

  // Member: fire
  cdr >> ros_message.fire;

  // Member: tracking
  cdr >> ros_message.tracking;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vision_interfaces
get_serialized_size(
  const vision_interfaces::msg::AutoAim & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: aim_yaw
  {
    size_t item_size = sizeof(ros_message.aim_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: aim_pitch
  {
    size_t item_size = sizeof(ros_message.aim_pitch);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: fire
  {
    size_t item_size = sizeof(ros_message.fire);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tracking
  {
    size_t item_size = sizeof(ros_message.tracking);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_vision_interfaces
max_serialized_size_AutoAim(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: aim_yaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: aim_pitch
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: fire
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: tracking
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = vision_interfaces::msg::AutoAim;
    is_plain =
      (
      offsetof(DataType, tracking) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _AutoAim__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const vision_interfaces::msg::AutoAim *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _AutoAim__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<vision_interfaces::msg::AutoAim *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _AutoAim__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const vision_interfaces::msg::AutoAim *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _AutoAim__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_AutoAim(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _AutoAim__callbacks = {
  "vision_interfaces::msg",
  "AutoAim",
  _AutoAim__cdr_serialize,
  _AutoAim__cdr_deserialize,
  _AutoAim__get_serialized_size,
  _AutoAim__max_serialized_size
};

static rosidl_message_type_support_t _AutoAim__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_AutoAim__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace vision_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_vision_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<vision_interfaces::msg::AutoAim>()
{
  return &vision_interfaces::msg::typesupport_fastrtps_cpp::_AutoAim__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, vision_interfaces, msg, AutoAim)() {
  return &vision_interfaces::msg::typesupport_fastrtps_cpp::_AutoAim__handle;
}

#ifdef __cplusplus
}
#endif
