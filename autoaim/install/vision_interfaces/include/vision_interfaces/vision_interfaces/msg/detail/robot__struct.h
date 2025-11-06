// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from vision_interfaces:msg/Robot.idl
// generated code does not contain a copyright notice

#ifndef VISION_INTERFACES__MSG__DETAIL__ROBOT__STRUCT_H_
#define VISION_INTERFACES__MSG__DETAIL__ROBOT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Robot in the package vision_interfaces.
typedef struct vision_interfaces__msg__Robot
{
  uint8_t mode;
  uint8_t foe_color;
  float self_yaw;
  float self_pitch;
  float muzzle_speed;
} vision_interfaces__msg__Robot;

// Struct for a sequence of vision_interfaces__msg__Robot.
typedef struct vision_interfaces__msg__Robot__Sequence
{
  vision_interfaces__msg__Robot * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} vision_interfaces__msg__Robot__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // VISION_INTERFACES__MSG__DETAIL__ROBOT__STRUCT_H_
