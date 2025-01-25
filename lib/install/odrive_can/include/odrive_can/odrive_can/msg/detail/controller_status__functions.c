// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from odrive_can:msg/ControllerStatus.idl
// generated code does not contain a copyright notice
#include "odrive_can/msg/detail/controller_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
odrive_can__msg__ControllerStatus__init(odrive_can__msg__ControllerStatus * msg)
{
  if (!msg) {
    return false;
  }
  // pos_estimate
  // vel_estimate
  // torque_target
  // torque_estimate
  // iq_setpoint
  // iq_measured
  // active_errors
  // axis_state
  // procedure_result
  // trajectory_done_flag
  return true;
}

void
odrive_can__msg__ControllerStatus__fini(odrive_can__msg__ControllerStatus * msg)
{
  if (!msg) {
    return;
  }
  // pos_estimate
  // vel_estimate
  // torque_target
  // torque_estimate
  // iq_setpoint
  // iq_measured
  // active_errors
  // axis_state
  // procedure_result
  // trajectory_done_flag
}

bool
odrive_can__msg__ControllerStatus__are_equal(const odrive_can__msg__ControllerStatus * lhs, const odrive_can__msg__ControllerStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pos_estimate
  if (lhs->pos_estimate != rhs->pos_estimate) {
    return false;
  }
  // vel_estimate
  if (lhs->vel_estimate != rhs->vel_estimate) {
    return false;
  }
  // torque_target
  if (lhs->torque_target != rhs->torque_target) {
    return false;
  }
  // torque_estimate
  if (lhs->torque_estimate != rhs->torque_estimate) {
    return false;
  }
  // iq_setpoint
  if (lhs->iq_setpoint != rhs->iq_setpoint) {
    return false;
  }
  // iq_measured
  if (lhs->iq_measured != rhs->iq_measured) {
    return false;
  }
  // active_errors
  if (lhs->active_errors != rhs->active_errors) {
    return false;
  }
  // axis_state
  if (lhs->axis_state != rhs->axis_state) {
    return false;
  }
  // procedure_result
  if (lhs->procedure_result != rhs->procedure_result) {
    return false;
  }
  // trajectory_done_flag
  if (lhs->trajectory_done_flag != rhs->trajectory_done_flag) {
    return false;
  }
  return true;
}

bool
odrive_can__msg__ControllerStatus__copy(
  const odrive_can__msg__ControllerStatus * input,
  odrive_can__msg__ControllerStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // pos_estimate
  output->pos_estimate = input->pos_estimate;
  // vel_estimate
  output->vel_estimate = input->vel_estimate;
  // torque_target
  output->torque_target = input->torque_target;
  // torque_estimate
  output->torque_estimate = input->torque_estimate;
  // iq_setpoint
  output->iq_setpoint = input->iq_setpoint;
  // iq_measured
  output->iq_measured = input->iq_measured;
  // active_errors
  output->active_errors = input->active_errors;
  // axis_state
  output->axis_state = input->axis_state;
  // procedure_result
  output->procedure_result = input->procedure_result;
  // trajectory_done_flag
  output->trajectory_done_flag = input->trajectory_done_flag;
  return true;
}

odrive_can__msg__ControllerStatus *
odrive_can__msg__ControllerStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  odrive_can__msg__ControllerStatus * msg = (odrive_can__msg__ControllerStatus *)allocator.allocate(sizeof(odrive_can__msg__ControllerStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(odrive_can__msg__ControllerStatus));
  bool success = odrive_can__msg__ControllerStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
odrive_can__msg__ControllerStatus__destroy(odrive_can__msg__ControllerStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    odrive_can__msg__ControllerStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
odrive_can__msg__ControllerStatus__Sequence__init(odrive_can__msg__ControllerStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  odrive_can__msg__ControllerStatus * data = NULL;

  if (size) {
    data = (odrive_can__msg__ControllerStatus *)allocator.zero_allocate(size, sizeof(odrive_can__msg__ControllerStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = odrive_can__msg__ControllerStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        odrive_can__msg__ControllerStatus__fini(&data[i - 1]);
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
odrive_can__msg__ControllerStatus__Sequence__fini(odrive_can__msg__ControllerStatus__Sequence * array)
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
      odrive_can__msg__ControllerStatus__fini(&array->data[i]);
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

odrive_can__msg__ControllerStatus__Sequence *
odrive_can__msg__ControllerStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  odrive_can__msg__ControllerStatus__Sequence * array = (odrive_can__msg__ControllerStatus__Sequence *)allocator.allocate(sizeof(odrive_can__msg__ControllerStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = odrive_can__msg__ControllerStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
odrive_can__msg__ControllerStatus__Sequence__destroy(odrive_can__msg__ControllerStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    odrive_can__msg__ControllerStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
odrive_can__msg__ControllerStatus__Sequence__are_equal(const odrive_can__msg__ControllerStatus__Sequence * lhs, const odrive_can__msg__ControllerStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!odrive_can__msg__ControllerStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
odrive_can__msg__ControllerStatus__Sequence__copy(
  const odrive_can__msg__ControllerStatus__Sequence * input,
  odrive_can__msg__ControllerStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(odrive_can__msg__ControllerStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    odrive_can__msg__ControllerStatus * data =
      (odrive_can__msg__ControllerStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!odrive_can__msg__ControllerStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          odrive_can__msg__ControllerStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!odrive_can__msg__ControllerStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
