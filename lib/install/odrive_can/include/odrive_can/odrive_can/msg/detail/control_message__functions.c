// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from odrive_can:msg/ControlMessage.idl
// generated code does not contain a copyright notice
#include "odrive_can/msg/detail/control_message__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
odrive_can__msg__ControlMessage__init(odrive_can__msg__ControlMessage * msg)
{
  if (!msg) {
    return false;
  }
  // control_mode
  // input_mode
  // input_pos
  // input_vel
  // input_torque
  return true;
}

void
odrive_can__msg__ControlMessage__fini(odrive_can__msg__ControlMessage * msg)
{
  if (!msg) {
    return;
  }
  // control_mode
  // input_mode
  // input_pos
  // input_vel
  // input_torque
}

bool
odrive_can__msg__ControlMessage__are_equal(const odrive_can__msg__ControlMessage * lhs, const odrive_can__msg__ControlMessage * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // control_mode
  if (lhs->control_mode != rhs->control_mode) {
    return false;
  }
  // input_mode
  if (lhs->input_mode != rhs->input_mode) {
    return false;
  }
  // input_pos
  if (lhs->input_pos != rhs->input_pos) {
    return false;
  }
  // input_vel
  if (lhs->input_vel != rhs->input_vel) {
    return false;
  }
  // input_torque
  if (lhs->input_torque != rhs->input_torque) {
    return false;
  }
  return true;
}

bool
odrive_can__msg__ControlMessage__copy(
  const odrive_can__msg__ControlMessage * input,
  odrive_can__msg__ControlMessage * output)
{
  if (!input || !output) {
    return false;
  }
  // control_mode
  output->control_mode = input->control_mode;
  // input_mode
  output->input_mode = input->input_mode;
  // input_pos
  output->input_pos = input->input_pos;
  // input_vel
  output->input_vel = input->input_vel;
  // input_torque
  output->input_torque = input->input_torque;
  return true;
}

odrive_can__msg__ControlMessage *
odrive_can__msg__ControlMessage__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  odrive_can__msg__ControlMessage * msg = (odrive_can__msg__ControlMessage *)allocator.allocate(sizeof(odrive_can__msg__ControlMessage), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(odrive_can__msg__ControlMessage));
  bool success = odrive_can__msg__ControlMessage__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
odrive_can__msg__ControlMessage__destroy(odrive_can__msg__ControlMessage * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    odrive_can__msg__ControlMessage__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
odrive_can__msg__ControlMessage__Sequence__init(odrive_can__msg__ControlMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  odrive_can__msg__ControlMessage * data = NULL;

  if (size) {
    data = (odrive_can__msg__ControlMessage *)allocator.zero_allocate(size, sizeof(odrive_can__msg__ControlMessage), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = odrive_can__msg__ControlMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        odrive_can__msg__ControlMessage__fini(&data[i - 1]);
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
odrive_can__msg__ControlMessage__Sequence__fini(odrive_can__msg__ControlMessage__Sequence * array)
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
      odrive_can__msg__ControlMessage__fini(&array->data[i]);
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

odrive_can__msg__ControlMessage__Sequence *
odrive_can__msg__ControlMessage__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  odrive_can__msg__ControlMessage__Sequence * array = (odrive_can__msg__ControlMessage__Sequence *)allocator.allocate(sizeof(odrive_can__msg__ControlMessage__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = odrive_can__msg__ControlMessage__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
odrive_can__msg__ControlMessage__Sequence__destroy(odrive_can__msg__ControlMessage__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    odrive_can__msg__ControlMessage__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
odrive_can__msg__ControlMessage__Sequence__are_equal(const odrive_can__msg__ControlMessage__Sequence * lhs, const odrive_can__msg__ControlMessage__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!odrive_can__msg__ControlMessage__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
odrive_can__msg__ControlMessage__Sequence__copy(
  const odrive_can__msg__ControlMessage__Sequence * input,
  odrive_can__msg__ControlMessage__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(odrive_can__msg__ControlMessage);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    odrive_can__msg__ControlMessage * data =
      (odrive_can__msg__ControlMessage *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!odrive_can__msg__ControlMessage__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          odrive_can__msg__ControlMessage__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!odrive_can__msg__ControlMessage__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
