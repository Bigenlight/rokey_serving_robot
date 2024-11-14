// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interface:srv/Order.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__ORDER__TRAITS_HPP_
#define CUSTOM_INTERFACE__SRV__DETAIL__ORDER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interface/srv/detail/order__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Order_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: table_number
  {
    out << "table_number: ";
    rosidl_generator_traits::value_to_yaml(msg.table_number, out);
    out << ", ";
  }

  // member: time
  {
    if (msg.time.size() == 0) {
      out << "time: []";
    } else {
      out << "time: [";
      size_t pending_items = msg.time.size();
      for (auto item : msg.time) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: menu
  {
    if (msg.menu.size() == 0) {
      out << "menu: []";
    } else {
      out << "menu: [";
      size_t pending_items = msg.menu.size();
      for (auto item : msg.menu) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Order_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: table_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "table_number: ";
    rosidl_generator_traits::value_to_yaml(msg.table_number, out);
    out << "\n";
  }

  // member: time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.time.size() == 0) {
      out << "time: []\n";
    } else {
      out << "time:\n";
      for (auto item : msg.time) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: menu
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.menu.size() == 0) {
      out << "menu: []\n";
    } else {
      out << "menu:\n";
      for (auto item : msg.menu) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Order_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_interface

namespace rosidl_generator_traits
{

[[deprecated("use custom_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interface::srv::Order_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interface::srv::Order_Request & msg)
{
  return custom_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interface::srv::Order_Request>()
{
  return "custom_interface::srv::Order_Request";
}

template<>
inline const char * name<custom_interface::srv::Order_Request>()
{
  return "custom_interface/srv/Order_Request";
}

template<>
struct has_fixed_size<custom_interface::srv::Order_Request>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_interface::srv::Order_Request>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_interface::srv::Order_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace custom_interface
{

namespace srv
{

inline void to_flow_style_yaml(
  const Order_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Order_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Order_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_interface

namespace rosidl_generator_traits
{

[[deprecated("use custom_interface::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interface::srv::Order_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interface::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interface::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interface::srv::Order_Response & msg)
{
  return custom_interface::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interface::srv::Order_Response>()
{
  return "custom_interface::srv::Order_Response";
}

template<>
inline const char * name<custom_interface::srv::Order_Response>()
{
  return "custom_interface/srv/Order_Response";
}

template<>
struct has_fixed_size<custom_interface::srv::Order_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_interface::srv::Order_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_interface::srv::Order_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<custom_interface::srv::Order>()
{
  return "custom_interface::srv::Order";
}

template<>
inline const char * name<custom_interface::srv::Order>()
{
  return "custom_interface/srv/Order";
}

template<>
struct has_fixed_size<custom_interface::srv::Order>
  : std::integral_constant<
    bool,
    has_fixed_size<custom_interface::srv::Order_Request>::value &&
    has_fixed_size<custom_interface::srv::Order_Response>::value
  >
{
};

template<>
struct has_bounded_size<custom_interface::srv::Order>
  : std::integral_constant<
    bool,
    has_bounded_size<custom_interface::srv::Order_Request>::value &&
    has_bounded_size<custom_interface::srv::Order_Response>::value
  >
{
};

template<>
struct is_service<custom_interface::srv::Order>
  : std::true_type
{
};

template<>
struct is_service_request<custom_interface::srv::Order_Request>
  : std::true_type
{
};

template<>
struct is_service_response<custom_interface::srv::Order_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__ORDER__TRAITS_HPP_
