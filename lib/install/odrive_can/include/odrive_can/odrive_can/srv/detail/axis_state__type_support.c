// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from odrive_can:srv/AxisState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "odrive_can/srv/detail/axis_state__rosidl_typesupport_introspection_c.h"
#include "odrive_can/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "odrive_can/srv/detail/axis_state__functions.h"
#include "odrive_can/srv/detail/axis_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void odrive_can__srv__AxisState_Request__rosidl_typesupport_introspection_c__AxisState_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  odrive_can__srv__AxisState_Request__init(message_memory);
}

void odrive_can__srv__AxisState_Request__rosidl_typesupport_introspection_c__AxisState_Request_fini_function(void * message_memory)
{
  odrive_can__srv__AxisState_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember odrive_can__srv__AxisState_Request__rosidl_typesupport_introspection_c__AxisState_Request_message_member_array[1] = {
  {
    "axis_requested_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(odrive_can__srv__AxisState_Request, axis_requested_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers odrive_can__srv__AxisState_Request__rosidl_typesupport_introspection_c__AxisState_Request_message_members = {
  "odrive_can__srv",  // message namespace
  "AxisState_Request",  // message name
  1,  // number of fields
  sizeof(odrive_can__srv__AxisState_Request),
  odrive_can__srv__AxisState_Request__rosidl_typesupport_introspection_c__AxisState_Request_message_member_array,  // message members
  odrive_can__srv__AxisState_Request__rosidl_typesupport_introspection_c__AxisState_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  odrive_can__srv__AxisState_Request__rosidl_typesupport_introspection_c__AxisState_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t odrive_can__srv__AxisState_Request__rosidl_typesupport_introspection_c__AxisState_Request_message_type_support_handle = {
  0,
  &odrive_can__srv__AxisState_Request__rosidl_typesupport_introspection_c__AxisState_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_odrive_can
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, odrive_can, srv, AxisState_Request)() {
  if (!odrive_can__srv__AxisState_Request__rosidl_typesupport_introspection_c__AxisState_Request_message_type_support_handle.typesupport_identifier) {
    odrive_can__srv__AxisState_Request__rosidl_typesupport_introspection_c__AxisState_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &odrive_can__srv__AxisState_Request__rosidl_typesupport_introspection_c__AxisState_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "odrive_can/srv/detail/axis_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "odrive_can/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "odrive_can/srv/detail/axis_state__functions.h"
// already included above
// #include "odrive_can/srv/detail/axis_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void odrive_can__srv__AxisState_Response__rosidl_typesupport_introspection_c__AxisState_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  odrive_can__srv__AxisState_Response__init(message_memory);
}

void odrive_can__srv__AxisState_Response__rosidl_typesupport_introspection_c__AxisState_Response_fini_function(void * message_memory)
{
  odrive_can__srv__AxisState_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember odrive_can__srv__AxisState_Response__rosidl_typesupport_introspection_c__AxisState_Response_message_member_array[3] = {
  {
    "active_errors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(odrive_can__srv__AxisState_Response, active_errors),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "axis_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(odrive_can__srv__AxisState_Response, axis_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "procedure_result",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(odrive_can__srv__AxisState_Response, procedure_result),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers odrive_can__srv__AxisState_Response__rosidl_typesupport_introspection_c__AxisState_Response_message_members = {
  "odrive_can__srv",  // message namespace
  "AxisState_Response",  // message name
  3,  // number of fields
  sizeof(odrive_can__srv__AxisState_Response),
  odrive_can__srv__AxisState_Response__rosidl_typesupport_introspection_c__AxisState_Response_message_member_array,  // message members
  odrive_can__srv__AxisState_Response__rosidl_typesupport_introspection_c__AxisState_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  odrive_can__srv__AxisState_Response__rosidl_typesupport_introspection_c__AxisState_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t odrive_can__srv__AxisState_Response__rosidl_typesupport_introspection_c__AxisState_Response_message_type_support_handle = {
  0,
  &odrive_can__srv__AxisState_Response__rosidl_typesupport_introspection_c__AxisState_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_odrive_can
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, odrive_can, srv, AxisState_Response)() {
  if (!odrive_can__srv__AxisState_Response__rosidl_typesupport_introspection_c__AxisState_Response_message_type_support_handle.typesupport_identifier) {
    odrive_can__srv__AxisState_Response__rosidl_typesupport_introspection_c__AxisState_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &odrive_can__srv__AxisState_Response__rosidl_typesupport_introspection_c__AxisState_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "odrive_can/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "odrive_can/srv/detail/axis_state__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers odrive_can__srv__detail__axis_state__rosidl_typesupport_introspection_c__AxisState_service_members = {
  "odrive_can__srv",  // service namespace
  "AxisState",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // odrive_can__srv__detail__axis_state__rosidl_typesupport_introspection_c__AxisState_Request_message_type_support_handle,
  NULL  // response message
  // odrive_can__srv__detail__axis_state__rosidl_typesupport_introspection_c__AxisState_Response_message_type_support_handle
};

static rosidl_service_type_support_t odrive_can__srv__detail__axis_state__rosidl_typesupport_introspection_c__AxisState_service_type_support_handle = {
  0,
  &odrive_can__srv__detail__axis_state__rosidl_typesupport_introspection_c__AxisState_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, odrive_can, srv, AxisState_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, odrive_can, srv, AxisState_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_odrive_can
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, odrive_can, srv, AxisState)() {
  if (!odrive_can__srv__detail__axis_state__rosidl_typesupport_introspection_c__AxisState_service_type_support_handle.typesupport_identifier) {
    odrive_can__srv__detail__axis_state__rosidl_typesupport_introspection_c__AxisState_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)odrive_can__srv__detail__axis_state__rosidl_typesupport_introspection_c__AxisState_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, odrive_can, srv, AxisState_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, odrive_can, srv, AxisState_Response)()->data;
  }

  return &odrive_can__srv__detail__axis_state__rosidl_typesupport_introspection_c__AxisState_service_type_support_handle;
}
