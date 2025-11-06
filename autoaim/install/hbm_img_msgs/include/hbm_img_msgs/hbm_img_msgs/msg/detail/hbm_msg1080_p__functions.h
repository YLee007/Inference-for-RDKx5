// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from hbm_img_msgs:msg/HbmMsg1080P.idl
// generated code does not contain a copyright notice

#ifndef HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__FUNCTIONS_H_
#define HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "hbm_img_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "hbm_img_msgs/msg/detail/hbm_msg1080_p__struct.h"

/// Initialize msg/HbmMsg1080P message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * hbm_img_msgs__msg__HbmMsg1080P
 * )) before or use
 * hbm_img_msgs__msg__HbmMsg1080P__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_hbm_img_msgs
bool
hbm_img_msgs__msg__HbmMsg1080P__init(hbm_img_msgs__msg__HbmMsg1080P * msg);

/// Finalize msg/HbmMsg1080P message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hbm_img_msgs
void
hbm_img_msgs__msg__HbmMsg1080P__fini(hbm_img_msgs__msg__HbmMsg1080P * msg);

/// Create msg/HbmMsg1080P message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * hbm_img_msgs__msg__HbmMsg1080P__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hbm_img_msgs
hbm_img_msgs__msg__HbmMsg1080P *
hbm_img_msgs__msg__HbmMsg1080P__create();

/// Destroy msg/HbmMsg1080P message.
/**
 * It calls
 * hbm_img_msgs__msg__HbmMsg1080P__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hbm_img_msgs
void
hbm_img_msgs__msg__HbmMsg1080P__destroy(hbm_img_msgs__msg__HbmMsg1080P * msg);

/// Check for msg/HbmMsg1080P message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hbm_img_msgs
bool
hbm_img_msgs__msg__HbmMsg1080P__are_equal(const hbm_img_msgs__msg__HbmMsg1080P * lhs, const hbm_img_msgs__msg__HbmMsg1080P * rhs);

/// Copy a msg/HbmMsg1080P message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hbm_img_msgs
bool
hbm_img_msgs__msg__HbmMsg1080P__copy(
  const hbm_img_msgs__msg__HbmMsg1080P * input,
  hbm_img_msgs__msg__HbmMsg1080P * output);

/// Initialize array of msg/HbmMsg1080P messages.
/**
 * It allocates the memory for the number of elements and calls
 * hbm_img_msgs__msg__HbmMsg1080P__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_hbm_img_msgs
bool
hbm_img_msgs__msg__HbmMsg1080P__Sequence__init(hbm_img_msgs__msg__HbmMsg1080P__Sequence * array, size_t size);

/// Finalize array of msg/HbmMsg1080P messages.
/**
 * It calls
 * hbm_img_msgs__msg__HbmMsg1080P__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hbm_img_msgs
void
hbm_img_msgs__msg__HbmMsg1080P__Sequence__fini(hbm_img_msgs__msg__HbmMsg1080P__Sequence * array);

/// Create array of msg/HbmMsg1080P messages.
/**
 * It allocates the memory for the array and calls
 * hbm_img_msgs__msg__HbmMsg1080P__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_hbm_img_msgs
hbm_img_msgs__msg__HbmMsg1080P__Sequence *
hbm_img_msgs__msg__HbmMsg1080P__Sequence__create(size_t size);

/// Destroy array of msg/HbmMsg1080P messages.
/**
 * It calls
 * hbm_img_msgs__msg__HbmMsg1080P__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_hbm_img_msgs
void
hbm_img_msgs__msg__HbmMsg1080P__Sequence__destroy(hbm_img_msgs__msg__HbmMsg1080P__Sequence * array);

/// Check for msg/HbmMsg1080P message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_hbm_img_msgs
bool
hbm_img_msgs__msg__HbmMsg1080P__Sequence__are_equal(const hbm_img_msgs__msg__HbmMsg1080P__Sequence * lhs, const hbm_img_msgs__msg__HbmMsg1080P__Sequence * rhs);

/// Copy an array of msg/HbmMsg1080P messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_hbm_img_msgs
bool
hbm_img_msgs__msg__HbmMsg1080P__Sequence__copy(
  const hbm_img_msgs__msg__HbmMsg1080P__Sequence * input,
  hbm_img_msgs__msg__HbmMsg1080P__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // HBM_IMG_MSGS__MSG__DETAIL__HBM_MSG1080_P__FUNCTIONS_H_
