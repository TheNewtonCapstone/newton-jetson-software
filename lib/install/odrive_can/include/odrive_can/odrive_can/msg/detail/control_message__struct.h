// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from odrive_can:msg/ControlMessage.idl
// generated code does not contain a copyright notice

#ifndef ODRIVE_CAN__MSG__DETAIL__CONTROL_MESSAGE__STRUCT_H_
#define ODRIVE_CAN__MSG__DETAIL__CONTROL_MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ControlMessage in the package odrive_can.
typedef struct odrive_can__msg__ControlMessage
{
  uint32_t control_mode;
  uint32_t input_mode;
  float input_pos;
  float input_vel;
  float input_torque;
} odrive_can__msg__ControlMessage;

// Struct for a sequence of odrive_can__msg__ControlMessage.
typedef struct odrive_can__msg__ControlMessage__Sequence
{
  odrive_can__msg__ControlMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} odrive_can__msg__ControlMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ODRIVE_CAN__MSG__DETAIL__CONTROL_MESSAGE__STRUCT_H_
