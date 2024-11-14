// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interface:srv/Order.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__ORDER__STRUCT_H_
#define CUSTOM_INTERFACE__SRV__DETAIL__ORDER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'time'
// Member 'menu'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Order in the package custom_interface.
typedef struct custom_interface__srv__Order_Request
{
  int8_t table_number;
  rosidl_runtime_c__String__Sequence time;
  rosidl_runtime_c__String__Sequence menu;
} custom_interface__srv__Order_Request;

// Struct for a sequence of custom_interface__srv__Order_Request.
typedef struct custom_interface__srv__Order_Request__Sequence
{
  custom_interface__srv__Order_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__srv__Order_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'response'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Order in the package custom_interface.
typedef struct custom_interface__srv__Order_Response
{
  rosidl_runtime_c__String__Sequence response;
} custom_interface__srv__Order_Response;

// Struct for a sequence of custom_interface__srv__Order_Response.
typedef struct custom_interface__srv__Order_Response__Sequence
{
  custom_interface__srv__Order_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interface__srv__Order_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__ORDER__STRUCT_H_
