// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from vision_interfaces:msg/Robot.idl
// generated code does not contain a copyright notice
#include "vision_interfaces/msg/detail/robot__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
vision_interfaces__msg__Robot__init(vision_interfaces__msg__Robot * msg)
{
  if (!msg) {
    return false;
  }
  // mode
  // foe_color
  // self_yaw
  // self_pitch
  // muzzle_speed
  return true;
}

void
vision_interfaces__msg__Robot__fini(vision_interfaces__msg__Robot * msg)
{
  if (!msg) {
    return;
  }
  // mode
  // foe_color
  // self_yaw
  // self_pitch
  // muzzle_speed
}

bool
vision_interfaces__msg__Robot__are_equal(const vision_interfaces__msg__Robot * lhs, const vision_interfaces__msg__Robot * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // foe_color
  if (lhs->foe_color != rhs->foe_color) {
    return false;
  }
  // self_yaw
  if (lhs->self_yaw != rhs->self_yaw) {
    return false;
  }
  // self_pitch
  if (lhs->self_pitch != rhs->self_pitch) {
    return false;
  }
  // muzzle_speed
  if (lhs->muzzle_speed != rhs->muzzle_speed) {
    return false;
  }
  return true;
}

bool
vision_interfaces__msg__Robot__copy(
  const vision_interfaces__msg__Robot * input,
  vision_interfaces__msg__Robot * output)
{
  if (!input || !output) {
    return false;
  }
  // mode
  output->mode = input->mode;
  // foe_color
  output->foe_color = input->foe_color;
  // self_yaw
  output->self_yaw = input->self_yaw;
  // self_pitch
  output->self_pitch = input->self_pitch;
  // muzzle_speed
  output->muzzle_speed = input->muzzle_speed;
  return true;
}

vision_interfaces__msg__Robot *
vision_interfaces__msg__Robot__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vision_interfaces__msg__Robot * msg = (vision_interfaces__msg__Robot *)allocator.allocate(sizeof(vision_interfaces__msg__Robot), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(vision_interfaces__msg__Robot));
  bool success = vision_interfaces__msg__Robot__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
vision_interfaces__msg__Robot__destroy(vision_interfaces__msg__Robot * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    vision_interfaces__msg__Robot__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
vision_interfaces__msg__Robot__Sequence__init(vision_interfaces__msg__Robot__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vision_interfaces__msg__Robot * data = NULL;

  if (size) {
    data = (vision_interfaces__msg__Robot *)allocator.zero_allocate(size, sizeof(vision_interfaces__msg__Robot), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = vision_interfaces__msg__Robot__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        vision_interfaces__msg__Robot__fini(&data[i - 1]);
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
vision_interfaces__msg__Robot__Sequence__fini(vision_interfaces__msg__Robot__Sequence * array)
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
      vision_interfaces__msg__Robot__fini(&array->data[i]);
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

vision_interfaces__msg__Robot__Sequence *
vision_interfaces__msg__Robot__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  vision_interfaces__msg__Robot__Sequence * array = (vision_interfaces__msg__Robot__Sequence *)allocator.allocate(sizeof(vision_interfaces__msg__Robot__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = vision_interfaces__msg__Robot__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
vision_interfaces__msg__Robot__Sequence__destroy(vision_interfaces__msg__Robot__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    vision_interfaces__msg__Robot__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
vision_interfaces__msg__Robot__Sequence__are_equal(const vision_interfaces__msg__Robot__Sequence * lhs, const vision_interfaces__msg__Robot__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!vision_interfaces__msg__Robot__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
vision_interfaces__msg__Robot__Sequence__copy(
  const vision_interfaces__msg__Robot__Sequence * input,
  vision_interfaces__msg__Robot__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(vision_interfaces__msg__Robot);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    vision_interfaces__msg__Robot * data =
      (vision_interfaces__msg__Robot *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!vision_interfaces__msg__Robot__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          vision_interfaces__msg__Robot__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!vision_interfaces__msg__Robot__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
