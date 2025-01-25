// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from odrive_can:srv/AxisState.idl
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
#include "odrive_can/srv/detail/axis_state__struct.h"
#include "odrive_can/srv/detail/axis_state__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool odrive_can__srv__axis_state__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[45];
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
    assert(strncmp("odrive_can.srv._axis_state.AxisState_Request", full_classname_dest, 44) == 0);
  }
  odrive_can__srv__AxisState_Request * ros_message = _ros_message;
  {  // axis_requested_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "axis_requested_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->axis_requested_state = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * odrive_can__srv__axis_state__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of AxisState_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("odrive_can.srv._axis_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "AxisState_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  odrive_can__srv__AxisState_Request * ros_message = (odrive_can__srv__AxisState_Request *)raw_ros_message;
  {  // axis_requested_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->axis_requested_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "axis_requested_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "odrive_can/srv/detail/axis_state__struct.h"
// already included above
// #include "odrive_can/srv/detail/axis_state__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool odrive_can__srv__axis_state__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("odrive_can.srv._axis_state.AxisState_Response", full_classname_dest, 45) == 0);
  }
  odrive_can__srv__AxisState_Response * ros_message = _ros_message;
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

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * odrive_can__srv__axis_state__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of AxisState_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("odrive_can.srv._axis_state");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "AxisState_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  odrive_can__srv__AxisState_Response * ros_message = (odrive_can__srv__AxisState_Response *)raw_ros_message;
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

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
