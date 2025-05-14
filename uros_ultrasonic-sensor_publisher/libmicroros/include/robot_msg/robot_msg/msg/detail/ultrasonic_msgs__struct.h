// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msg:msg/UltrasonicMsgs.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSG__MSG__DETAIL__ULTRASONIC_MSGS__STRUCT_H_
#define ROBOT_MSG__MSG__DETAIL__ULTRASONIC_MSGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'ranges'
#include "sensor_msgs/msg/detail/range__struct.h"

/// Struct defined in msg/UltrasonicMsgs in the package robot_msg.
typedef struct robot_msg__msg__UltrasonicMsgs
{
  sensor_msgs__msg__Range__Sequence ranges;
} robot_msg__msg__UltrasonicMsgs;

// Struct for a sequence of robot_msg__msg__UltrasonicMsgs.
typedef struct robot_msg__msg__UltrasonicMsgs__Sequence
{
  robot_msg__msg__UltrasonicMsgs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msg__msg__UltrasonicMsgs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSG__MSG__DETAIL__ULTRASONIC_MSGS__STRUCT_H_
