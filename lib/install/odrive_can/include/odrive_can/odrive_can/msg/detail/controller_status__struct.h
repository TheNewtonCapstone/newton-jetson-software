// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from odrive_can:msg/ControllerStatus.idl
// generated code does not contain a copyright notice

#ifndef ODRIVE_CAN__MSG__DETAIL__CONTROLLER_STATUS__STRUCT_H_
#define ODRIVE_CAN__MSG__DETAIL__CONTROLLER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ControllerStatus in the package odrive_can.
typedef struct odrive_can__msg__ControllerStatus
{
  float pos_estimate;
  float vel_estimate;
  float torque_target;
  float torque_estimate;
  float iq_setpoint;
  float iq_measured;
  uint32_t active_errors;
  uint8_t axis_state;
  uint8_t procedure_result;
  bool trajectory_done_flag;
} odrive_can__msg__ControllerStatus;

// Struct for a sequence of odrive_can__msg__ControllerStatus.
typedef struct odrive_can__msg__ControllerStatus__Sequence
{
  odrive_can__msg__ControllerStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} odrive_can__msg__ControllerStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ODRIVE_CAN__MSG__DETAIL__CONTROLLER_STATUS__STRUCT_H_
