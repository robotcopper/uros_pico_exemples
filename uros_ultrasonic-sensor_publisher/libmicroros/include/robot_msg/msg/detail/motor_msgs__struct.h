// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msg:msg/MotorMsgs.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_MSG__MSG__DETAIL__MOTOR_MSGS__STRUCT_H_
#define ROBOT_MSG__MSG__DETAIL__MOTOR_MSGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MotorMsgs in the package robot_msg.
typedef struct robot_msg__msg__MotorMsgs
{
  float motor1;
  float motor2;
  float motor3;
} robot_msg__msg__MotorMsgs;

// Struct for a sequence of robot_msg__msg__MotorMsgs.
typedef struct robot_msg__msg__MotorMsgs__Sequence
{
  robot_msg__msg__MotorMsgs * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msg__msg__MotorMsgs__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSG__MSG__DETAIL__MOTOR_MSGS__STRUCT_H_
