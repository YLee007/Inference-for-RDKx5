// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hbm_img_msgs:msg/HbmMsg1080P.idl
// generated code does not contain a copyright notice
#include "hbm_img_msgs/msg/detail/hbm_msg1080_p__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `time_stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
hbm_img_msgs__msg__HbmMsg1080P__init(hbm_img_msgs__msg__HbmMsg1080P * msg)
{
  if (!msg) {
    return false;
  }
  // index
  // time_stamp
  if (!builtin_interfaces__msg__Time__init(&msg->time_stamp)) {
    hbm_img_msgs__msg__HbmMsg1080P__fini(msg);
    return false;
  }
  // height
  // width
  // data_size
  // step
  // encoding
  // data
  return true;
}

void
hbm_img_msgs__msg__HbmMsg1080P__fini(hbm_img_msgs__msg__HbmMsg1080P * msg)
{
  if (!msg) {
    return;
  }
  // index
  // time_stamp
  builtin_interfaces__msg__Time__fini(&msg->time_stamp);
  // height
  // width
  // data_size
  // step
  // encoding
  // data
}

bool
hbm_img_msgs__msg__HbmMsg1080P__are_equal(const hbm_img_msgs__msg__HbmMsg1080P * lhs, const hbm_img_msgs__msg__HbmMsg1080P * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // index
  if (lhs->index != rhs->index) {
    return false;
  }
  // time_stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->time_stamp), &(rhs->time_stamp)))
  {
    return false;
  }
  // height
  if (lhs->height != rhs->height) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // data_size
  if (lhs->data_size != rhs->data_size) {
    return false;
  }
  // step
  if (lhs->step != rhs->step) {
    return false;
  }
  // encoding
  for (size_t i = 0; i < 12; ++i) {
    if (lhs->encoding[i] != rhs->encoding[i]) {
      return false;
    }
  }
  // data
  for (size_t i = 0; i < 6220800; ++i) {
    if (lhs->data[i] != rhs->data[i]) {
      return false;
    }
  }
  return true;
}

bool
hbm_img_msgs__msg__HbmMsg1080P__copy(
  const hbm_img_msgs__msg__HbmMsg1080P * input,
  hbm_img_msgs__msg__HbmMsg1080P * output)
{
  if (!input || !output) {
    return false;
  }
  // index
  output->index = input->index;
  // time_stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->time_stamp), &(output->time_stamp)))
  {
    return false;
  }
  // height
  output->height = input->height;
  // width
  output->width = input->width;
  // data_size
  output->data_size = input->data_size;
  // step
  output->step = input->step;
  // encoding
  for (size_t i = 0; i < 12; ++i) {
    output->encoding[i] = input->encoding[i];
  }
  // data
  for (size_t i = 0; i < 6220800; ++i) {
    output->data[i] = input->data[i];
  }
  return true;
}

hbm_img_msgs__msg__HbmMsg1080P *
hbm_img_msgs__msg__HbmMsg1080P__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hbm_img_msgs__msg__HbmMsg1080P * msg = (hbm_img_msgs__msg__HbmMsg1080P *)allocator.allocate(sizeof(hbm_img_msgs__msg__HbmMsg1080P), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hbm_img_msgs__msg__HbmMsg1080P));
  bool success = hbm_img_msgs__msg__HbmMsg1080P__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hbm_img_msgs__msg__HbmMsg1080P__destroy(hbm_img_msgs__msg__HbmMsg1080P * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hbm_img_msgs__msg__HbmMsg1080P__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hbm_img_msgs__msg__HbmMsg1080P__Sequence__init(hbm_img_msgs__msg__HbmMsg1080P__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hbm_img_msgs__msg__HbmMsg1080P * data = NULL;

  if (size) {
    data = (hbm_img_msgs__msg__HbmMsg1080P *)allocator.zero_allocate(size, sizeof(hbm_img_msgs__msg__HbmMsg1080P), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hbm_img_msgs__msg__HbmMsg1080P__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hbm_img_msgs__msg__HbmMsg1080P__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
hbm_img_msgs__msg__HbmMsg1080P__Sequence__fini(hbm_img_msgs__msg__HbmMsg1080P__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      hbm_img_msgs__msg__HbmMsg1080P__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

hbm_img_msgs__msg__HbmMsg1080P__Sequence *
hbm_img_msgs__msg__HbmMsg1080P__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hbm_img_msgs__msg__HbmMsg1080P__Sequence * array = (hbm_img_msgs__msg__HbmMsg1080P__Sequence *)allocator.allocate(sizeof(hbm_img_msgs__msg__HbmMsg1080P__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hbm_img_msgs__msg__HbmMsg1080P__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hbm_img_msgs__msg__HbmMsg1080P__Sequence__destroy(hbm_img_msgs__msg__HbmMsg1080P__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hbm_img_msgs__msg__HbmMsg1080P__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hbm_img_msgs__msg__HbmMsg1080P__Sequence__are_equal(const hbm_img_msgs__msg__HbmMsg1080P__Sequence * lhs, const hbm_img_msgs__msg__HbmMsg1080P__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hbm_img_msgs__msg__HbmMsg1080P__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hbm_img_msgs__msg__HbmMsg1080P__Sequence__copy(
  const hbm_img_msgs__msg__HbmMsg1080P__Sequence * input,
  hbm_img_msgs__msg__HbmMsg1080P__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hbm_img_msgs__msg__HbmMsg1080P);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hbm_img_msgs__msg__HbmMsg1080P * data =
      (hbm_img_msgs__msg__HbmMsg1080P *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hbm_img_msgs__msg__HbmMsg1080P__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hbm_img_msgs__msg__HbmMsg1080P__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hbm_img_msgs__msg__HbmMsg1080P__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
