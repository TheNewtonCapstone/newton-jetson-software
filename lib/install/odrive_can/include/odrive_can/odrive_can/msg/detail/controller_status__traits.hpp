// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from odrive_can:msg/ControllerStatus.idl
// generated code does not contain a copyright notice

#ifndef ODRIVE_CAN__MSG__DETAIL__CONTROLLER_STATUS__TRAITS_HPP_
#define ODRIVE_CAN__MSG__DETAIL__CONTROLLER_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "odrive_can/msg/detail/controller_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace odrive_can
{

namespace msg
{

inline void to_flow_style_yaml(
  const ControllerStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: pos_estimate
  {
    out << "pos_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_estimate, out);
    out << ", ";
  }

  // member: vel_estimate
  {
    out << "vel_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_estimate, out);
    out << ", ";
  }

  // member: torque_target
  {
    out << "torque_target: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_target, out);
    out << ", ";
  }

  // member: torque_estimate
  {
    out << "torque_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_estimate, out);
    out << ", ";
  }

  // member: iq_setpoint
  {
    out << "iq_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.iq_setpoint, out);
    out << ", ";
  }

  // member: iq_measured
  {
    out << "iq_measured: ";
    rosidl_generator_traits::value_to_yaml(msg.iq_measured, out);
    out << ", ";
  }

  // member: active_errors
  {
    out << "active_errors: ";
    rosidl_generator_traits::value_to_yaml(msg.active_errors, out);
    out << ", ";
  }

  // member: axis_state
  {
    out << "axis_state: ";
    rosidl_generator_traits::value_to_yaml(msg.axis_state, out);
    out << ", ";
  }

  // member: procedure_result
  {
    out << "procedure_result: ";
    rosidl_generator_traits::value_to_yaml(msg.procedure_result, out);
    out << ", ";
  }

  // member: trajectory_done_flag
  {
    out << "trajectory_done_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_done_flag, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ControllerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pos_estimate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.pos_estimate, out);
    out << "\n";
  }

  // member: vel_estimate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_estimate, out);
    out << "\n";
  }

  // member: torque_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torque_target: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_target, out);
    out << "\n";
  }

  // member: torque_estimate
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "torque_estimate: ";
    rosidl_generator_traits::value_to_yaml(msg.torque_estimate, out);
    out << "\n";
  }

  // member: iq_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "iq_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.iq_setpoint, out);
    out << "\n";
  }

  // member: iq_measured
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "iq_measured: ";
    rosidl_generator_traits::value_to_yaml(msg.iq_measured, out);
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

  // member: axis_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "axis_state: ";
    rosidl_generator_traits::value_to_yaml(msg.axis_state, out);
    out << "\n";
  }

  // member: procedure_result
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "procedure_result: ";
    rosidl_generator_traits::value_to_yaml(msg.procedure_result, out);
    out << "\n";
  }

  // member: trajectory_done_flag
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "trajectory_done_flag: ";
    rosidl_generator_traits::value_to_yaml(msg.trajectory_done_flag, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ControllerStatus & msg, bool use_flow_style = false)
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
  const odrive_can::msg::ControllerStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  odrive_can::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use odrive_can::msg::to_yaml() instead")]]
inline std::string to_yaml(const odrive_can::msg::ControllerStatus & msg)
{
  return odrive_can::msg::to_yaml(msg);
}

template<>
inline const char * data_type<odrive_can::msg::ControllerStatus>()
{
  return "odrive_can::msg::ControllerStatus";
}

template<>
inline const char * name<odrive_can::msg::ControllerStatus>()
{
  return "odrive_can/msg/ControllerStatus";
}

template<>
struct has_fixed_size<odrive_can::msg::ControllerStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<odrive_can::msg::ControllerStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<odrive_can::msg::ControllerStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ODRIVE_CAN__MSG__DETAIL__CONTROLLER_STATUS__TRAITS_HPP_
