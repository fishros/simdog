// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from champ_msgs:msg/PointArray.idl
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
#include "champ_msgs/msg/detail/point_array__struct.h"
#include "champ_msgs/msg/detail/point_array__functions.h"

bool champ_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * champ_msgs__msg__point__convert_to_py(void * raw_ros_message);
bool champ_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * champ_msgs__msg__point__convert_to_py(void * raw_ros_message);
bool champ_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * champ_msgs__msg__point__convert_to_py(void * raw_ros_message);
bool champ_msgs__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * champ_msgs__msg__point__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool champ_msgs__msg__point_array__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[39];
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
    assert(strncmp("champ_msgs.msg._point_array.PointArray", full_classname_dest, 38) == 0);
  }
  champ_msgs__msg__PointArray * ros_message = _ros_message;
  {  // lf
    PyObject * field = PyObject_GetAttrString(_pymsg, "lf");
    if (!field) {
      return false;
    }
    if (!champ_msgs__msg__point__convert_from_py(field, &ros_message->lf)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // rf
    PyObject * field = PyObject_GetAttrString(_pymsg, "rf");
    if (!field) {
      return false;
    }
    if (!champ_msgs__msg__point__convert_from_py(field, &ros_message->rf)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // lh
    PyObject * field = PyObject_GetAttrString(_pymsg, "lh");
    if (!field) {
      return false;
    }
    if (!champ_msgs__msg__point__convert_from_py(field, &ros_message->lh)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // rh
    PyObject * field = PyObject_GetAttrString(_pymsg, "rh");
    if (!field) {
      return false;
    }
    if (!champ_msgs__msg__point__convert_from_py(field, &ros_message->rh)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * champ_msgs__msg__point_array__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PointArray */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("champ_msgs.msg._point_array");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PointArray");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  champ_msgs__msg__PointArray * ros_message = (champ_msgs__msg__PointArray *)raw_ros_message;
  {  // lf
    PyObject * field = NULL;
    field = champ_msgs__msg__point__convert_to_py(&ros_message->lf);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "lf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rf
    PyObject * field = NULL;
    field = champ_msgs__msg__point__convert_to_py(&ros_message->rf);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "rf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // lh
    PyObject * field = NULL;
    field = champ_msgs__msg__point__convert_to_py(&ros_message->lh);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "lh", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rh
    PyObject * field = NULL;
    field = champ_msgs__msg__point__convert_to_py(&ros_message->rh);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "rh", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
