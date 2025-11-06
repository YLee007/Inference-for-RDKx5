// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vision_interfaces:msg/AutoAim.idl
// generated code does not contain a copyright notice

#ifndef VISION_INTERFACES__MSG__DETAIL__AUTO_AIM__STRUCT_H_
#define VISION_INTERFACES__MSG__DETAIL__AUTO_AIM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/AutoAim in the package vision_interfaces.
typedef struct vision_interfaces__msg__AutoAim
{
  float aim_yaw;
  float aim_pitch;
  uint8_t fire;
  uint8_t tracking;
} vision_interfaces__msg__AutoAim;

// Struct for a sequence of vision_interfaces__msg__AutoAim.
typedef struct vision_interfaces__msg__AutoAim__Sequence
{
  vision_interfaces__msg__AutoAim * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vision_interfaces__msg__AutoAim__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VISION_INTERFACES__MSG__DETAIL__AUTO_AIM__STRUCT_H_
