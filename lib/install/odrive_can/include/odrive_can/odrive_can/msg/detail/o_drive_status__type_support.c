// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from odrive_can:msg/ODriveStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "odrive_can/msg/detail/o_drive_status__rosidl_typesupport_introspection_c.h"
#include "odrive_can/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "odrive_can/msg/detail/o_drive_status__functions.h"
#include "odrive_can/msg/detail/o_drive_status__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void odrive_can__msg__ODriveStatus__rosidl_typesupport_introspection_c__ODriveStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  odrive_can__msg__ODriveStatus__init(message_memory);
}

void odrive_can__msg__ODriveStatus__rosidl_typesupport_introspection_c__ODriveStatus_fini_function(void * message_memory)
{
  odrive_can__msg__ODriveStatus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember odrive_can__msg__ODriveStatus__rosidl_typesupport_introspection_c__ODriveStatus_message_member_array[6] = {
  {
    "bus_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(odrive_can__msg__ODriveStatus, bus_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bus_current",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(odrive_can__msg__ODriveStatus, bus_current),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "fet_temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(odrive_can__msg__ODriveStatus, fet_temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(odrive_can__msg__ODriveStatus, motor_temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "active_errors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(odrive_can__msg__ODriveStatus, active_errors),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "disarm_reason",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(odrive_can__msg__ODriveStatus, disarm_reason),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers odrive_can__msg__ODriveStatus__rosidl_typesupport_introspection_c__ODriveStatus_message_members = {
  "odrive_can__msg",  // message namespace
  "ODriveStatus",  // message name
  6,  // number of fields
  sizeof(odrive_can__msg__ODriveStatus),
  odrive_can__msg__ODriveStatus__rosidl_typesupport_introspection_c__ODriveStatus_message_member_array,  // message members
  odrive_can__msg__ODriveStatus__rosidl_typesupport_introspection_c__ODriveStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  odrive_can__msg__ODriveStatus__rosidl_typesupport_introspection_c__ODriveStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t odrive_can__msg__ODriveStatus__rosidl_typesupport_introspection_c__ODriveStatus_message_type_support_handle = {
  0,
  &odrive_can__msg__ODriveStatus__rosidl_typesupport_introspection_c__ODriveStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_odrive_can
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, odrive_can, msg, ODriveStatus)() {
  if (!odrive_can__msg__ODriveStatus__rosidl_typesupport_introspection_c__ODriveStatus_message_type_support_handle.typesupport_identifier) {
    odrive_can__msg__ODriveStatus__rosidl_typesupport_introspection_c__ODriveStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &odrive_can__msg__ODriveStatus__rosidl_typesupport_introspection_c__ODriveStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
