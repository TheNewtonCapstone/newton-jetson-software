// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from odrive_can:msg/ControllerStatus.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "odrive_can/msg/detail/controller_status__struct.h"
#include "odrive_can/msg/detail/controller_status__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool odrive_can__msg__controller_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[51];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("odrive_can.msg._controller_status.ControllerStatus", full_classname_dest, 50) == 0);
  }
  odrive_can__msg__ControllerStatus * ros_message = _ros_message;
  {  // pos_estimate
    PyObject * field = PyObject_GetAttrString(_pymsg, "pos_estimate");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->pos_estimate = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // vel_estimate
    PyObject * field = PyObject_GetAttrString(_pymsg, "vel_estimate");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->vel_estimate = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // torque_target
    PyObject * field = PyObject_GetAttrString(_pymsg, "torque_target");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->torque_target = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // torque_estimate
    PyObject * field = PyObject_GetAttrString(_pymsg, "torque_estimate");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->torque_estimate = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // iq_setpoint
    PyObject * field = PyObject_GetAttrString(_pymsg, "iq_setpoint");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->iq_setpoint = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // iq_measured
    PyObject * field = PyObject_GetAttrString(_pymsg, "iq_measured");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->iq_measured = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // active_errors
    PyObject * field = PyObject_GetAttrString(_pymsg, "active_errors");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->active_errors = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // axis_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "axis_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->axis_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // procedure_result
    PyObject * field = PyObject_GetAttrString(_pymsg, "procedure_result");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->procedure_result = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // trajectory_done_flag
    PyObject * field = PyObject_GetAttrString(_pymsg, "trajectory_done_flag");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->trajectory_done_flag = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * odrive_can__msg__controller_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ControllerStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("odrive_can.msg._controller_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ControllerStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  odrive_can__msg__ControllerStatus * ros_message = (odrive_can__msg__ControllerStatus *)raw_ros_message;
  {  // pos_estimate
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->pos_estimate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pos_estimate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // vel_estimate
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->vel_estimate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "vel_estimate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // torque_target
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->torque_target);
    {
      int rc = PyObject_SetAttrString(_pymessage, "torque_target", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // torque_estimate
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->torque_estimate);
    {
      int rc = PyObject_SetAttrString(_pymessage, "torque_estimate", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // iq_setpoint
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->iq_setpoint);
    {
      int rc = PyObject_SetAttrString(_pymessage, "iq_setpoint", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // iq_measured
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->iq_measured);
    {
      int rc = PyObject_SetAttrString(_pymessage, "iq_measured", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // active_errors
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->active_errors);
    {
      int rc = PyObject_SetAttrString(_pymessage, "active_errors", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // axis_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->axis_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "axis_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // procedure_result
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->procedure_result);
    {
      int rc = PyObject_SetAttrString(_pymessage, "procedure_result", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // trajectory_done_flag
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->trajectory_done_flag ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "trajectory_done_flag", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
