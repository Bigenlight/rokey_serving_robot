// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interface:srv/Order.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACE__SRV__DETAIL__ORDER__BUILDER_HPP_
#define CUSTOM_INTERFACE__SRV__DETAIL__ORDER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interface/srv/detail/order__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interface
{

namespace srv
{

namespace builder
{

class Init_Order_Request_menu
{
public:
  explicit Init_Order_Request_menu(::custom_interface::srv::Order_Request & msg)
  : msg_(msg)
  {}
  ::custom_interface::srv::Order_Request menu(::custom_interface::srv::Order_Request::_menu_type arg)
  {
    msg_.menu = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::srv::Order_Request msg_;
};

class Init_Order_Request_time
{
public:
  explicit Init_Order_Request_time(::custom_interface::srv::Order_Request & msg)
  : msg_(msg)
  {}
  Init_Order_Request_menu time(::custom_interface::srv::Order_Request::_time_type arg)
  {
    msg_.time = std::move(arg);
    return Init_Order_Request_menu(msg_);
  }

private:
  ::custom_interface::srv::Order_Request msg_;
};

class Init_Order_Request_table_number
{
public:
  Init_Order_Request_table_number()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Order_Request_time table_number(::custom_interface::srv::Order_Request::_table_number_type arg)
  {
    msg_.table_number = std::move(arg);
    return Init_Order_Request_time(msg_);
  }

private:
  ::custom_interface::srv::Order_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::srv::Order_Request>()
{
  return custom_interface::srv::builder::Init_Order_Request_table_number();
}

}  // namespace custom_interface


namespace custom_interface
{

namespace srv
{

namespace builder
{

class Init_Order_Response_response
{
public:
  Init_Order_Response_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interface::srv::Order_Response response(::custom_interface::srv::Order_Response::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interface::srv::Order_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interface::srv::Order_Response>()
{
  return custom_interface::srv::builder::Init_Order_Response_response();
}

}  // namespace custom_interface

#endif  // CUSTOM_INTERFACE__SRV__DETAIL__ORDER__BUILDER_HPP_
