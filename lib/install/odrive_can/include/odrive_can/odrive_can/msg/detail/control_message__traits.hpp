// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from odrive_can:msg/ControlMessage.idl
// generated code does not contain a copyright notice

#ifndef ODRIVE_CAN__MSG__DETAIL__CONTROL_MESSAGE__TRAITS_HPP_
#define ODRIVE_CAN__MSG__DETAIL__CONTROL_MESSAGE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "odrive_can/msg/detail/control_message__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace odrive_can
{

namespace msg
{

inline void to_flow_style_yaml(
  const ControlMessage & msg,
  std::ostream & out)
{
  out << "{";
  // member: control_mode
  {
    out << "control_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.control_mode, out);
    out << ", ";
  }

  // member: input_mode
  {
    out << "input_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.input_mode, out);
    out << ", ";
  }

  // member: input_pos
  {
    out << "input_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.input_pos, out);
    out << ", ";
  }

  // member: input_vel
  {
    out << "input_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.input_vel, out);
    out << ", ";
  }

  // member: input_torque
  {
    out << "input_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.input_torque, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControlMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: control_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "control_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.control_mode, out);
    out << "\n";
  }

  // member: input_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "input_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.input_mode, out);
    out << "\n";
  }

  // member: input_pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "input_pos: ";
    rosidl_generator_traits::value_to_yaml(msg.input_pos, out);
    out << "\n";
  }

  // member: input_vel
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "input_vel: ";
    rosidl_generator_traits::value_to_yaml(msg.input_vel, out);
    out << "\n";
  }

  // member: input_torque
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "input_torque: ";
    rosidl_generator_traits::value_to_yaml(msg.input_torque, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControlMessage & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace odrive_can

namespace rosidl_generator_traits
{

[[deprecated("use odrive_can::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const odrive_can::msg::ControlMessage & msg,
  std::ostream & out, size_t indentation = 0)
{
  odrive_can::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use odrive_can::msg::to_yaml() instead")]]
inline std::string to_yaml(const odrive_can::msg::ControlMessage & msg)
{
  return odrive_can::msg::to_yaml(msg);
}

template<>
inline const char * data_type<odrive_can::msg::ControlMessage>()
{
  return "odrive_can::msg::ControlMessage";
}

template<>
inline const char * name<odrive_can::msg::ControlMessage>()
{
  return "odrive_can/msg/ControlMessage";
}

template<>
struct has_fixed_size<odrive_can::msg::ControlMessage>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<odrive_can::msg::ControlMessage>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<odrive_can::msg::ControlMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ODRIVE_CAN__MSG__DETAIL__CONTROL_MESSAGE__TRAITS_HPP_
