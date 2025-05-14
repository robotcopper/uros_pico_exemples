// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_msg:msg/UltrasonicMsgs.idl
// generated code does not contain a copyright notice
#include "robot_msg/msg/detail/ultrasonic_msgs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `ranges`
#include "sensor_msgs/msg/detail/range__functions.h"

bool
robot_msg__msg__UltrasonicMsgs__init(robot_msg__msg__UltrasonicMsgs * msg)
{
  if (!msg) {
    return false;
  }
  // ranges
  if (!sensor_msgs__msg__Range__Sequence__init(&msg->ranges, 0)) {
    robot_msg__msg__UltrasonicMsgs__fini(msg);
    return false;
  }
  return true;
}

void
robot_msg__msg__UltrasonicMsgs__fini(robot_msg__msg__UltrasonicMsgs * msg)
{
  if (!msg) {
    return;
  }
  // ranges
  sensor_msgs__msg__Range__Sequence__fini(&msg->ranges);
}

bool
robot_msg__msg__UltrasonicMsgs__are_equal(const robot_msg__msg__UltrasonicMsgs * lhs, const robot_msg__msg__UltrasonicMsgs * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ranges
  if (!sensor_msgs__msg__Range__Sequence__are_equal(
      &(lhs->ranges), &(rhs->ranges)))
  {
    return false;
  }
  return true;
}

bool
robot_msg__msg__UltrasonicMsgs__copy(
  const robot_msg__msg__UltrasonicMsgs * input,
  robot_msg__msg__UltrasonicMsgs * output)
{
  if (!input || !output) {
    return false;
  }
  // ranges
  if (!sensor_msgs__msg__Range__Sequence__copy(
      &(input->ranges), &(output->ranges)))
  {
    return false;
  }
  return true;
}

robot_msg__msg__UltrasonicMsgs *
robot_msg__msg__UltrasonicMsgs__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msg__msg__UltrasonicMsgs * msg = (robot_msg__msg__UltrasonicMsgs *)allocator.allocate(sizeof(robot_msg__msg__UltrasonicMsgs), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_msg__msg__UltrasonicMsgs));
  bool success = robot_msg__msg__UltrasonicMsgs__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_msg__msg__UltrasonicMsgs__destroy(robot_msg__msg__UltrasonicMsgs * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_msg__msg__UltrasonicMsgs__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_msg__msg__UltrasonicMsgs__Sequence__init(robot_msg__msg__UltrasonicMsgs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msg__msg__UltrasonicMsgs * data = NULL;

  if (size) {
    data = (robot_msg__msg__UltrasonicMsgs *)allocator.zero_allocate(size, sizeof(robot_msg__msg__UltrasonicMsgs), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_msg__msg__UltrasonicMsgs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_msg__msg__UltrasonicMsgs__fini(&data[i - 1]);
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
robot_msg__msg__UltrasonicMsgs__Sequence__fini(robot_msg__msg__UltrasonicMsgs__Sequence * array)
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
      robot_msg__msg__UltrasonicMsgs__fini(&array->data[i]);
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

robot_msg__msg__UltrasonicMsgs__Sequence *
robot_msg__msg__UltrasonicMsgs__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msg__msg__UltrasonicMsgs__Sequence * array = (robot_msg__msg__UltrasonicMsgs__Sequence *)allocator.allocate(sizeof(robot_msg__msg__UltrasonicMsgs__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_msg__msg__UltrasonicMsgs__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_msg__msg__UltrasonicMsgs__Sequence__destroy(robot_msg__msg__UltrasonicMsgs__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_msg__msg__UltrasonicMsgs__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_msg__msg__UltrasonicMsgs__Sequence__are_equal(const robot_msg__msg__UltrasonicMsgs__Sequence * lhs, const robot_msg__msg__UltrasonicMsgs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_msg__msg__UltrasonicMsgs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_msg__msg__UltrasonicMsgs__Sequence__copy(
  const robot_msg__msg__UltrasonicMsgs__Sequence * input,
  robot_msg__msg__UltrasonicMsgs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_msg__msg__UltrasonicMsgs);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_msg__msg__UltrasonicMsgs * data =
      (robot_msg__msg__UltrasonicMsgs *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_msg__msg__UltrasonicMsgs__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_msg__msg__UltrasonicMsgs__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_msg__msg__UltrasonicMsgs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
