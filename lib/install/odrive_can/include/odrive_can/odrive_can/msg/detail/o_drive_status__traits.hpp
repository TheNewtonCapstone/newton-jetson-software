// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from odrive_can:msg/ODriveStatus.idl
// generated code does not contain a copyright notice

#ifndef ODRIVE_CAN__MSG__DETAIL__O_DRIVE_STATUS__TRAITS_HPP_
#define ODRIVE_CAN__MSG__DETAIL__O_DRIVE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "odrive_can/msg/detail/o_drive_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace odrive_can
{

namespace msg
{

inline void to_flow_style_yaml(
  const ODriveStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: bus_voltage
  {
    out << "bus_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.bus_voltage, out);
    out << ", ";
  }

  // member: bus_current
  {
    out << "bus_current: ";
    rosidl_generator_traits::value_to_yaml(msg.bus_current, out);
    out << ", ";
  }

  // member: fet_temperature
  {
    out << "fet_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.fet_temperature, out);
    out << ", ";
  }

  // member: motor_temperature
  {
    out << "motor_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_temperature, out);
    out << ", ";
  }

  // member: active_errors
  {
    out << "active_errors: ";
    rosidl_generator_traits::value_to_yaml(msg.active_errors, out);
    out << ", ";
  }

  // member: disarm_reason
  {
    out << "disarm_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.disarm_reason, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ODriveStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: bus_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bus_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.bus_voltage, out);
    out << "\n";
  }

  // member: bus_current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bus_current: ";
    rosidl_generator_traits::value_to_yaml(msg.bus_current, out);
    out << "\n";
  }

  // member: fet_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "fet_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.fet_temperature, out);
    out << "\n";
  }

  // member: motor_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_temperature, out);
    out << "\n";
  }

  // member: active_errors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "active_errors: ";
    rosidl_generator_traits::value_to_yaml(msg.active_errors, out);
    out << "\n";
  }

  // member: disarm_reason
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "disarm_reason: ";
    rosidl_generator_traits::value_to_yaml(msg.disarm_reason, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ODriveStatus & msg, bool use_flow_style = false)
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
  const odrive_can::msg::ODriveStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  odrive_can::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use odrive_can::msg::to_yaml() instead")]]
inline std::string to_yaml(const odrive_can::msg::ODriveStatus & msg)
{
  return odrive_can::msg::to_yaml(msg);
}

template<>
inline const char * data_type<odrive_can::msg::ODriveStatus>()
{
  return "odrive_can::msg::ODriveStatus";
}

template<>
inline const char * name<odrive_can::msg::ODriveStatus>()
{
  return "odrive_can/msg/ODriveStatus";
}

template<>
struct has_fixed_size<odrive_can::msg::ODriveStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<odrive_can::msg::ODriveStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<odrive_can::msg::ODriveStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ODRIVE_CAN__MSG__DETAIL__O_DRIVE_STATUS__TRAITS_HPP_
