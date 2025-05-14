// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_msg:msg/UltrasonicMsgs.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_msg/msg/detail/ultrasonic_msgs__rosidl_typesupport_introspection_c.h"
#include "robot_msg/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_msg/msg/detail/ultrasonic_msgs__functions.h"
#include "robot_msg/msg/detail/ultrasonic_msgs__struct.h"


// Include directives for member types
// Member `ranges`
#include "sensor_msgs/msg/range.h"
// Member `ranges`
#include "sensor_msgs/msg/detail/range__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__UltrasonicMsgs_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msg__msg__UltrasonicMsgs__init(message_memory);
}

void robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__UltrasonicMsgs_fini_function(void * message_memory)
{
  robot_msg__msg__UltrasonicMsgs__fini(message_memory);
}

size_t robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__size_function__UltrasonicMsgs__ranges(
  const void * untyped_member)
{
  const sensor_msgs__msg__Range__Sequence * member =
    (const sensor_msgs__msg__Range__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__get_const_function__UltrasonicMsgs__ranges(
  const void * untyped_member, size_t index)
{
  const sensor_msgs__msg__Range__Sequence * member =
    (const sensor_msgs__msg__Range__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__get_function__UltrasonicMsgs__ranges(
  void * untyped_member, size_t index)
{
  sensor_msgs__msg__Range__Sequence * member =
    (sensor_msgs__msg__Range__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__fetch_function__UltrasonicMsgs__ranges(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sensor_msgs__msg__Range * item =
    ((const sensor_msgs__msg__Range *)
    robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__get_const_function__UltrasonicMsgs__ranges(untyped_member, index));
  sensor_msgs__msg__Range * value =
    (sensor_msgs__msg__Range *)(untyped_value);
  *value = *item;
}

void robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__assign_function__UltrasonicMsgs__ranges(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sensor_msgs__msg__Range * item =
    ((sensor_msgs__msg__Range *)
    robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__get_function__UltrasonicMsgs__ranges(untyped_member, index));
  const sensor_msgs__msg__Range * value =
    (const sensor_msgs__msg__Range *)(untyped_value);
  *item = *value;
}

bool robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__resize_function__UltrasonicMsgs__ranges(
  void * untyped_member, size_t size)
{
  sensor_msgs__msg__Range__Sequence * member =
    (sensor_msgs__msg__Range__Sequence *)(untyped_member);
  sensor_msgs__msg__Range__Sequence__fini(member);
  return sensor_msgs__msg__Range__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__UltrasonicMsgs_message_member_array[1] = {
  {
    "ranges",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msg__msg__UltrasonicMsgs, ranges),  // bytes offset in struct
    NULL,  // default value
    robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__size_function__UltrasonicMsgs__ranges,  // size() function pointer
    robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__get_const_function__UltrasonicMsgs__ranges,  // get_const(index) function pointer
    robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__get_function__UltrasonicMsgs__ranges,  // get(index) function pointer
    robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__fetch_function__UltrasonicMsgs__ranges,  // fetch(index, &value) function pointer
    robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__assign_function__UltrasonicMsgs__ranges,  // assign(index, value) function pointer
    robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__resize_function__UltrasonicMsgs__ranges  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__UltrasonicMsgs_message_members = {
  "robot_msg__msg",  // message namespace
  "UltrasonicMsgs",  // message name
  1,  // number of fields
  sizeof(robot_msg__msg__UltrasonicMsgs),
  robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__UltrasonicMsgs_message_member_array,  // message members
  robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__UltrasonicMsgs_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__UltrasonicMsgs_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__UltrasonicMsgs_message_type_support_handle = {
  0,
  &robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__UltrasonicMsgs_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msg
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msg, msg, UltrasonicMsgs)() {
  robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__UltrasonicMsgs_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, Range)();
  if (!robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__UltrasonicMsgs_message_type_support_handle.typesupport_identifier) {
    robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__UltrasonicMsgs_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msg__msg__UltrasonicMsgs__rosidl_typesupport_introspection_c__UltrasonicMsgs_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
