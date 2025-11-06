// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from hbm_img_msgs:msg/HbmMsg1080P.idl
// generated code does not contain a copyright notice

#ifndef HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__STRUCT_H_
#define HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MAX_SIZE'.
enum
{
  hbm_img_msgs__msg__HbmMsg1080P__MAX_SIZE = 6220800ul
};

// Include directives for member types
// Member 'time_stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/HbmMsg1080P in the package hbm_img_msgs.
/**
  * Copyright (c) 2024，D-Robotics.
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  *     http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
 */
typedef struct hbm_img_msgs__msg__HbmMsg1080P
{
  int32_t index;
  builtin_interfaces__msg__Time time_stamp;
  uint32_t height;
  uint32_t width;
  uint32_t data_size;
  uint32_t step;
  /// 这个字段用于标识图像格式
  uint8_t encoding[12];
  /// 存储图像数据，大小为6220800字节，适用于1080P图像的NV12格式数据
  uint8_t data[6220800];
} hbm_img_msgs__msg__HbmMsg1080P;

// Struct for a sequence of hbm_img_msgs__msg__HbmMsg1080P.
typedef struct hbm_img_msgs__msg__HbmMsg1080P__Sequence
{
  hbm_img_msgs__msg__HbmMsg1080P * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} hbm_img_msgs__msg__HbmMsg1080P__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__STRUCT_H_
