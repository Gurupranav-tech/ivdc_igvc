// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ublox_msgs:msg/RxmSFRBX.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ublox_msgs/msg/detail/rxm_sfrbx__rosidl_typesupport_introspection_c.h"
#include "ublox_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ublox_msgs/msg/detail/rxm_sfrbx__functions.h"
#include "ublox_msgs/msg/detail/rxm_sfrbx__struct.h"


// Include directives for member types
// Member `dwrd`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__RxmSFRBX_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ublox_msgs__msg__RxmSFRBX__init(message_memory);
}

void ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__RxmSFRBX_fini_function(void * message_memory)
{
  ublox_msgs__msg__RxmSFRBX__fini(message_memory);
}

size_t ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__size_function__RxmSFRBX__dwrd(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return member->size;
}

const void * ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__get_const_function__RxmSFRBX__dwrd(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__get_function__RxmSFRBX__dwrd(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__fetch_function__RxmSFRBX__dwrd(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__get_const_function__RxmSFRBX__dwrd(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__assign_function__RxmSFRBX__dwrd(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__get_function__RxmSFRBX__dwrd(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

bool ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__resize_function__RxmSFRBX__dwrd(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  rosidl_runtime_c__uint32__Sequence__fini(member);
  return rosidl_runtime_c__uint32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__RxmSFRBX_message_member_array[9] = {
  {
    "gnss_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_msgs__msg__RxmSFRBX, gnss_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sv_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_msgs__msg__RxmSFRBX, sv_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reserved0",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_msgs__msg__RxmSFRBX, reserved0),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "freq_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_msgs__msg__RxmSFRBX, freq_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_words",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_msgs__msg__RxmSFRBX, num_words),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "chn",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_msgs__msg__RxmSFRBX, chn),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "version",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_msgs__msg__RxmSFRBX, version),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "reserved1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_msgs__msg__RxmSFRBX, reserved1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "dwrd",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ublox_msgs__msg__RxmSFRBX, dwrd),  // bytes offset in struct
    NULL,  // default value
    ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__size_function__RxmSFRBX__dwrd,  // size() function pointer
    ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__get_const_function__RxmSFRBX__dwrd,  // get_const(index) function pointer
    ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__get_function__RxmSFRBX__dwrd,  // get(index) function pointer
    ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__fetch_function__RxmSFRBX__dwrd,  // fetch(index, &value) function pointer
    ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__assign_function__RxmSFRBX__dwrd,  // assign(index, value) function pointer
    ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__resize_function__RxmSFRBX__dwrd  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__RxmSFRBX_message_members = {
  "ublox_msgs__msg",  // message namespace
  "RxmSFRBX",  // message name
  9,  // number of fields
  sizeof(ublox_msgs__msg__RxmSFRBX),
  ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__RxmSFRBX_message_member_array,  // message members
  ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__RxmSFRBX_init_function,  // function to initialize message memory (memory has to be allocated)
  ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__RxmSFRBX_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__RxmSFRBX_message_type_support_handle = {
  0,
  &ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__RxmSFRBX_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ublox_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ublox_msgs, msg, RxmSFRBX)() {
  if (!ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__RxmSFRBX_message_type_support_handle.typesupport_identifier) {
    ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__RxmSFRBX_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ublox_msgs__msg__RxmSFRBX__rosidl_typesupport_introspection_c__RxmSFRBX_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
