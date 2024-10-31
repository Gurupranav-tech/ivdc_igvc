// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ublox_msgs:msg/MonGNSS.idl
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
#include "ublox_msgs/msg/detail/mon_gnss__struct.h"
#include "ublox_msgs/msg/detail/mon_gnss__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ublox_msgs__msg__mon_gnss__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[33];
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
    assert(strncmp("ublox_msgs.msg._mon_gnss.MonGNSS", full_classname_dest, 32) == 0);
  }
  ublox_msgs__msg__MonGNSS * ros_message = _ros_message;
  {  // version
    PyObject * field = PyObject_GetAttrString(_pymsg, "version");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->version = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // supported
    PyObject * field = PyObject_GetAttrString(_pymsg, "supported");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->supported = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // default_gnss
    PyObject * field = PyObject_GetAttrString(_pymsg, "default_gnss");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->default_gnss = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // enabled
    PyObject * field = PyObject_GetAttrString(_pymsg, "enabled");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->enabled = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // simultaneous
    PyObject * field = PyObject_GetAttrString(_pymsg, "simultaneous");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->simultaneous = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // reserved1
    PyObject * field = PyObject_GetAttrString(_pymsg, "reserved1");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_UINT8);
      Py_ssize_t size = 3;
      uint8_t * dest = ros_message->reserved1;
      for (Py_ssize_t i = 0; i < size; ++i) {
        uint8_t tmp = *(npy_uint8 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(uint8_t));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ublox_msgs__msg__mon_gnss__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MonGNSS */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ublox_msgs.msg._mon_gnss");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MonGNSS");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ublox_msgs__msg__MonGNSS * ros_message = (ublox_msgs__msg__MonGNSS *)raw_ros_message;
  {  // version
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->version);
    {
      int rc = PyObject_SetAttrString(_pymessage, "version", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // supported
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->supported);
    {
      int rc = PyObject_SetAttrString(_pymessage, "supported", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // default_gnss
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->default_gnss);
    {
      int rc = PyObject_SetAttrString(_pymessage, "default_gnss", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // enabled
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->enabled);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enabled", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // simultaneous
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->simultaneous);
    {
      int rc = PyObject_SetAttrString(_pymessage, "simultaneous", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reserved1
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "reserved1");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_UINT8);
    assert(sizeof(npy_uint8) == sizeof(uint8_t));
    npy_uint8 * dst = (npy_uint8 *)PyArray_GETPTR1(seq_field, 0);
    uint8_t * src = &(ros_message->reserved1[0]);
    memcpy(dst, src, 3 * sizeof(uint8_t));
    Py_DECREF(field);
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
