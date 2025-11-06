// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from vision_interfaces:msg/AutoAim.idl
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
#include "vision_interfaces/msg/detail/auto_aim__struct.h"
#include "vision_interfaces/msg/detail/auto_aim__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool vision_interfaces__msg__auto_aim__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[40];
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
    assert(strncmp("vision_interfaces.msg._auto_aim.AutoAim", full_classname_dest, 39) == 0);
  }
  vision_interfaces__msg__AutoAim * ros_message = _ros_message;
  {  // aim_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "aim_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->aim_yaw = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // aim_pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "aim_pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->aim_pitch = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // fire
    PyObject * field = PyObject_GetAttrString(_pymsg, "fire");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->fire = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // tracking
    PyObject * field = PyObject_GetAttrString(_pymsg, "tracking");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->tracking = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * vision_interfaces__msg__auto_aim__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of AutoAim */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("vision_interfaces.msg._auto_aim");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "AutoAim");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  vision_interfaces__msg__AutoAim * ros_message = (vision_interfaces__msg__AutoAim *)raw_ros_message;
  {  // aim_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->aim_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "aim_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // aim_pitch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->aim_pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "aim_pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // fire
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->fire);
    {
      int rc = PyObject_SetAttrString(_pymessage, "fire", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tracking
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->tracking);
    {
      int rc = PyObject_SetAttrString(_pymessage, "tracking", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
