// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from odrive_can:srv/AxisState.idl
// generated code does not contain a copyright notice

#ifndef ODRIVE_CAN__SRV__DETAIL__AXIS_STATE__STRUCT_H_
#define ODRIVE_CAN__SRV__DETAIL__AXIS_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/AxisState in the package odrive_can.
typedef struct odrive_can__srv__AxisState_Request
{
  uint32_t axis_requested_state;
} odrive_can__srv__AxisState_Request;

// Struct for a sequence of odrive_can__srv__AxisState_Request.
typedef struct odrive_can__srv__AxisState_Request__Sequence
{
  odrive_can__srv__AxisState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} odrive_can__srv__AxisState_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/AxisState in the package odrive_can.
typedef struct odrive_can__srv__AxisState_Response
{
  uint32_t active_errors;
  uint8_t axis_state;
  uint8_t procedure_result;
} odrive_can__srv__AxisState_Response;

// Struct for a sequence of odrive_can__srv__AxisState_Response.
typedef struct odrive_can__srv__AxisState_Response__Sequence
{
  odrive_can__srv__AxisState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} odrive_can__srv__AxisState_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ODRIVE_CAN__SRV__DETAIL__AXIS_STATE__STRUCT_H_
