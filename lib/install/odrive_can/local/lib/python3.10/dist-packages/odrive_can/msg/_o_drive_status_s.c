// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from odrive_can:msg/ODriveStatus.idl
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
#include "odrive_can/msg/detail/o_drive_status__struct.h"
#include "odrive_can/msg/detail/o_drive_status__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool odrive_can__msg__o_drive_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[44];
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
    assert(strncmp("odrive_can.msg._o_drive_status.ODriveStatus", full_classname_dest, 43) == 0);
  }
  odrive_can__msg__ODriveStatus * ros_message = _ros_message;
  {  // bus_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "bus_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bus_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bus_current
    PyObject * field = PyObject_GetAttrString(_pymsg, "bus_current");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bus_current = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fet_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "fet_temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->fet_temperature = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // motor_temperature
    PyObject * field = PyObject_GetAttrString(_pymsg, "motor_temperature");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->motor_temperature = (float)PyFloat_AS_DOUBLE(field);
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
  {  // disarm_reason
    PyObject * field = PyObject_GetAttrString(_pymsg, "disarm_reason");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->disarm_reason = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * odrive_can__msg__o_drive_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ODriveStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("odrive_can.msg._o_drive_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ODriveStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  odrive_can__msg__ODriveStatus * ros_message = (odrive_can__msg__ODriveStatus *)raw_ros_message;
  {  // bus_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bus_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bus_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bus_current
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bus_current);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bus_current", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fet_temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->fet_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fet_temperature", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // motor_temperature
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->motor_temperature);
    {
      int rc = PyObject_SetAttrString(_pymessage, "motor_temperature", field);
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
  {  // disarm_reason
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->disarm_reason);
    {
      int rc = PyObject_SetAttrString(_pymessage, "disarm_reason", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
