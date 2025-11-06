// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from vision_interfaces:msg/Robot.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "vision_interfaces/msg/detail/robot__rosidl_typesupport_introspection_c.h"
#include "vision_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "vision_interfaces/msg/detail/robot__functions.h"
#include "vision_interfaces/msg/detail/robot__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void vision_interfaces__msg__Robot__rosidl_typesupport_introspection_c__Robot_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  vision_interfaces__msg__Robot__init(message_memory);
}

void vision_interfaces__msg__Robot__rosidl_typesupport_introspection_c__Robot_fini_function(void * message_memory)
{
  vision_interfaces__msg__Robot__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember vision_interfaces__msg__Robot__rosidl_typesupport_introspection_c__Robot_message_member_array[5] = {
  {
    "mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_interfaces__msg__Robot, mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "foe_color",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_interfaces__msg__Robot, foe_color),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "self_yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_interfaces__msg__Robot, self_yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "self_pitch",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_interfaces__msg__Robot, self_pitch),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "muzzle_speed",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(vision_interfaces__msg__Robot, muzzle_speed),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers vision_interfaces__msg__Robot__rosidl_typesupport_introspection_c__Robot_message_members = {
  "vision_interfaces__msg",  // message namespace
  "Robot",  // message name
  5,  // number of fields
  sizeof(vision_interfaces__msg__Robot),
  vision_interfaces__msg__Robot__rosidl_typesupport_introspection_c__Robot_message_member_array,  // message members
  vision_interfaces__msg__Robot__rosidl_typesupport_introspection_c__Robot_init_function,  // function to initialize message memory (memory has to be allocated)
  vision_interfaces__msg__Robot__rosidl_typesupport_introspection_c__Robot_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t vision_interfaces__msg__Robot__rosidl_typesupport_introspection_c__Robot_message_type_support_handle = {
  0,
  &vision_interfaces__msg__Robot__rosidl_typesupport_introspection_c__Robot_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_vision_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, vision_interfaces, msg, Robot)() {
  if (!vision_interfaces__msg__Robot__rosidl_typesupport_introspection_c__Robot_message_type_support_handle.typesupport_identifier) {
    vision_interfaces__msg__Robot__rosidl_typesupport_introspection_c__Robot_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &vision_interfaces__msg__Robot__rosidl_typesupport_introspection_c__Robot_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
