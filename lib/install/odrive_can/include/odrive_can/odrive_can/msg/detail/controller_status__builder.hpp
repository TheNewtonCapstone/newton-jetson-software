// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from odrive_can:msg/ControllerStatus.idl
// generated code does not contain a copyright notice

#ifndef ODRIVE_CAN__MSG__DETAIL__CONTROLLER_STATUS__BUILDER_HPP_
#define ODRIVE_CAN__MSG__DETAIL__CONTROLLER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "odrive_can/msg/detail/controller_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace odrive_can
{

namespace msg
{

namespace builder
{

class Init_ControllerStatus_trajectory_done_flag
{
public:
  explicit Init_ControllerStatus_trajectory_done_flag(::odrive_can::msg::ControllerStatus & msg)
  : msg_(msg)
  {}
  ::odrive_can::msg::ControllerStatus trajectory_done_flag(::odrive_can::msg::ControllerStatus::_trajectory_done_flag_type arg)
  {
    msg_.trajectory_done_flag = std::move(arg);
    return std::move(msg_);
  }

private:
  ::odrive_can::msg::ControllerStatus msg_;
};

class Init_ControllerStatus_procedure_result
{
public:
  explicit Init_ControllerStatus_procedure_result(::odrive_can::msg::ControllerStatus & msg)
  : msg_(msg)
  {}
  Init_ControllerStatus_trajectory_done_flag procedure_result(::odrive_can::msg::ControllerStatus::_procedure_result_type arg)
  {
    msg_.procedure_result = std::move(arg);
    return Init_ControllerStatus_trajectory_done_flag(msg_);
  }

private:
  ::odrive_can::msg::ControllerStatus msg_;
};

class Init_ControllerStatus_axis_state
{
public:
  explicit Init_ControllerStatus_axis_state(::odrive_can::msg::ControllerStatus & msg)
  : msg_(msg)
  {}
  Init_ControllerStatus_procedure_result axis_state(::odrive_can::msg::ControllerStatus::_axis_state_type arg)
  {
    msg_.axis_state = std::move(arg);
    return Init_ControllerStatus_procedure_result(msg_);
  }

private:
  ::odrive_can::msg::ControllerStatus msg_;
};

class Init_ControllerStatus_active_errors
{
public:
  explicit Init_ControllerStatus_active_errors(::odrive_can::msg::ControllerStatus & msg)
  : msg_(msg)
  {}
  Init_ControllerStatus_axis_state active_errors(::odrive_can::msg::ControllerStatus::_active_errors_type arg)
  {
    msg_.active_errors = std::move(arg);
    return Init_ControllerStatus_axis_state(msg_);
  }

private:
  ::odrive_can::msg::ControllerStatus msg_;
};

class Init_ControllerStatus_iq_measured
{
public:
  explicit Init_ControllerStatus_iq_measured(::odrive_can::msg::ControllerStatus & msg)
  : msg_(msg)
  {}
  Init_ControllerStatus_active_errors iq_measured(::odrive_can::msg::ControllerStatus::_iq_measured_type arg)
  {
    msg_.iq_measured = std::move(arg);
    return Init_ControllerStatus_active_errors(msg_);
  }

private:
  ::odrive_can::msg::ControllerStatus msg_;
};

class Init_ControllerStatus_iq_setpoint
{
public:
  explicit Init_ControllerStatus_iq_setpoint(::odrive_can::msg::ControllerStatus & msg)
  : msg_(msg)
  {}
  Init_ControllerStatus_iq_measured iq_setpoint(::odrive_can::msg::ControllerStatus::_iq_setpoint_type arg)
  {
    msg_.iq_setpoint = std::move(arg);
    return Init_ControllerStatus_iq_measured(msg_);
  }

private:
  ::odrive_can::msg::ControllerStatus msg_;
};

class Init_ControllerStatus_torque_estimate
{
public:
  explicit Init_ControllerStatus_torque_estimate(::odrive_can::msg::ControllerStatus & msg)
  : msg_(msg)
  {}
  Init_ControllerStatus_iq_setpoint torque_estimate(::odrive_can::msg::ControllerStatus::_torque_estimate_type arg)
  {
    msg_.torque_estimate = std::move(arg);
    return Init_ControllerStatus_iq_setpoint(msg_);
  }

private:
  ::odrive_can::msg::ControllerStatus msg_;
};

class Init_ControllerStatus_torque_target
{
public:
  explicit Init_ControllerStatus_torque_target(::odrive_can::msg::ControllerStatus & msg)
  : msg_(msg)
  {}
  Init_ControllerStatus_torque_estimate torque_target(::odrive_can::msg::ControllerStatus::_torque_target_type arg)
  {
    msg_.torque_target = std::move(arg);
    return Init_ControllerStatus_torque_estimate(msg_);
  }

private:
  ::odrive_can::msg::ControllerStatus msg_;
};

class Init_ControllerStatus_vel_estimate
{
public:
  explicit Init_ControllerStatus_vel_estimate(::odrive_can::msg::ControllerStatus & msg)
  : msg_(msg)
  {}
  Init_ControllerStatus_torque_target vel_estimate(::odrive_can::msg::ControllerStatus::_vel_estimate_type arg)
  {
    msg_.vel_estimate = std::move(arg);
    return Init_ControllerStatus_torque_target(msg_);
  }

private:
  ::odrive_can::msg::ControllerStatus msg_;
};

class Init_ControllerStatus_pos_estimate
{
public:
  Init_ControllerStatus_pos_estimate()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControllerStatus_vel_estimate pos_estimate(::odrive_can::msg::ControllerStatus::_pos_estimate_type arg)
  {
    msg_.pos_estimate = std::move(arg);
    return Init_ControllerStatus_vel_estimate(msg_);
  }

private:
  ::odrive_can::msg::ControllerStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::odrive_can::msg::ControllerStatus>()
{
  return odrive_can::msg::builder::Init_ControllerStatus_pos_estimate();
}

}  // namespace odrive_can

#endif  // ODRIVE_CAN__MSG__DETAIL__CONTROLLER_STATUS__BUILDER_HPP_
