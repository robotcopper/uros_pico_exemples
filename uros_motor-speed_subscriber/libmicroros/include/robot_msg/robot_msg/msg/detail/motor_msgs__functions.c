// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_msg:msg/MotorMsgs.idl
// generated code does not contain a copyright notice
#include "robot_msg/msg/detail/motor_msgs__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
robot_msg__msg__MotorMsgs__init(robot_msg__msg__MotorMsgs * msg)
{
  if (!msg) {
    return false;
  }
  // motor1
  // motor2
  // motor3
  return true;
}

void
robot_msg__msg__MotorMsgs__fini(robot_msg__msg__MotorMsgs * msg)
{
  if (!msg) {
    return;
  }
  // motor1
  // motor2
  // motor3
}

bool
robot_msg__msg__MotorMsgs__are_equal(const robot_msg__msg__MotorMsgs * lhs, const robot_msg__msg__MotorMsgs * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // motor1
  if (lhs->motor1 != rhs->motor1) {
    return false;
  }
  // motor2
  if (lhs->motor2 != rhs->motor2) {
    return false;
  }
  // motor3
  if (lhs->motor3 != rhs->motor3) {
    return false;
  }
  return true;
}

bool
robot_msg__msg__MotorMsgs__copy(
  const robot_msg__msg__MotorMsgs * input,
  robot_msg__msg__MotorMsgs * output)
{
  if (!input || !output) {
    return false;
  }
  // motor1
  output->motor1 = input->motor1;
  // motor2
  output->motor2 = input->motor2;
  // motor3
  output->motor3 = input->motor3;
  return true;
}

robot_msg__msg__MotorMsgs *
robot_msg__msg__MotorMsgs__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msg__msg__MotorMsgs * msg = (robot_msg__msg__MotorMsgs *)allocator.allocate(sizeof(robot_msg__msg__MotorMsgs), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_msg__msg__MotorMsgs));
  bool success = robot_msg__msg__MotorMsgs__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_msg__msg__MotorMsgs__destroy(robot_msg__msg__MotorMsgs * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_msg__msg__MotorMsgs__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_msg__msg__MotorMsgs__Sequence__init(robot_msg__msg__MotorMsgs__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msg__msg__MotorMsgs * data = NULL;

  if (size) {
    data = (robot_msg__msg__MotorMsgs *)allocator.zero_allocate(size, sizeof(robot_msg__msg__MotorMsgs), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_msg__msg__MotorMsgs__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_msg__msg__MotorMsgs__fini(&data[i - 1]);
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
robot_msg__msg__MotorMsgs__Sequence__fini(robot_msg__msg__MotorMsgs__Sequence * array)
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
      robot_msg__msg__MotorMsgs__fini(&array->data[i]);
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

robot_msg__msg__MotorMsgs__Sequence *
robot_msg__msg__MotorMsgs__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_msg__msg__MotorMsgs__Sequence * array = (robot_msg__msg__MotorMsgs__Sequence *)allocator.allocate(sizeof(robot_msg__msg__MotorMsgs__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_msg__msg__MotorMsgs__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_msg__msg__MotorMsgs__Sequence__destroy(robot_msg__msg__MotorMsgs__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_msg__msg__MotorMsgs__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_msg__msg__MotorMsgs__Sequence__are_equal(const robot_msg__msg__MotorMsgs__Sequence * lhs, const robot_msg__msg__MotorMsgs__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_msg__msg__MotorMsgs__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_msg__msg__MotorMsgs__Sequence__copy(
  const robot_msg__msg__MotorMsgs__Sequence * input,
  robot_msg__msg__MotorMsgs__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_msg__msg__MotorMsgs);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_msg__msg__MotorMsgs * data =
      (robot_msg__msg__MotorMsgs *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_msg__msg__MotorMsgs__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_msg__msg__MotorMsgs__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_msg__msg__MotorMsgs__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
