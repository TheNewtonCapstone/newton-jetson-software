// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from odrive_can:msg/ODriveStatus.idl
// generated code does not contain a copyright notice

#ifndef ODRIVE_CAN__MSG__DETAIL__O_DRIVE_STATUS__BUILDER_HPP_
#define ODRIVE_CAN__MSG__DETAIL__O_DRIVE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "odrive_can/msg/detail/o_drive_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace odrive_can
{

namespace msg
{

namespace builder
{

class Init_ODriveStatus_disarm_reason
{
public:
  explicit Init_ODriveStatus_disarm_reason(::odrive_can::msg::ODriveStatus & msg)
  : msg_(msg)
  {}
  ::odrive_can::msg::ODriveStatus disarm_reason(::odrive_can::msg::ODriveStatus::_disarm_reason_type arg)
  {
    msg_.disarm_reason = std::move(arg);
    return std::move(msg_);
  }

private:
  ::odrive_can::msg::ODriveStatus msg_;
};

class Init_ODriveStatus_active_errors
{
public:
  explicit Init_ODriveStatus_active_errors(::odrive_can::msg::ODriveStatus & msg)
  : msg_(msg)
  {}
  Init_ODriveStatus_disarm_reason active_errors(::odrive_can::msg::ODriveStatus::_active_errors_type arg)
  {
    msg_.active_errors = std::move(arg);
    return Init_ODriveStatus_disarm_reason(msg_);
  }

private:
  ::odrive_can::msg::ODriveStatus msg_;
};

class Init_ODriveStatus_motor_temperature
{
public:
  explicit Init_ODriveStatus_motor_temperature(::odrive_can::msg::ODriveStatus & msg)
  : msg_(msg)
  {}
  Init_ODriveStatus_active_errors motor_temperature(::odrive_can::msg::ODriveStatus::_motor_temperature_type arg)
  {
    msg_.motor_temperature = std::move(arg);
    return Init_ODriveStatus_active_errors(msg_);
  }

private:
  ::odrive_can::msg::ODriveStatus msg_;
};

class Init_ODriveStatus_fet_temperature
{
public:
  explicit Init_ODriveStatus_fet_temperature(::odrive_can::msg::ODriveStatus & msg)
  : msg_(msg)
  {}
  Init_ODriveStatus_motor_temperature fet_temperature(::odrive_can::msg::ODriveStatus::_fet_temperature_type arg)
  {
    msg_.fet_temperature = std::move(arg);
    return Init_ODriveStatus_motor_temperature(msg_);
  }

private:
  ::odrive_can::msg::ODriveStatus msg_;
};

class Init_ODriveStatus_bus_current
{
public:
  explicit Init_ODriveStatus_bus_current(::odrive_can::msg::ODriveStatus & msg)
  : msg_(msg)
  {}
  Init_ODriveStatus_fet_temperature bus_current(::odrive_can::msg::ODriveStatus::_bus_current_type arg)
  {
    msg_.bus_current = std::move(arg);
    return Init_ODriveStatus_fet_temperature(msg_);
  }

private:
  ::odrive_can::msg::ODriveStatus msg_;
};

class Init_ODriveStatus_bus_voltage
{
public:
  Init_ODriveStatus_bus_voltage()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ODriveStatus_bus_current bus_voltage(::odrive_can::msg::ODriveStatus::_bus_voltage_type arg)
  {
    msg_.bus_voltage = std::move(arg);
    return Init_ODriveStatus_bus_current(msg_);
  }

private:
  ::odrive_can::msg::ODriveStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::odrive_can::msg::ODriveStatus>()
{
  return odrive_can::msg::builder::Init_ODriveStatus_bus_voltage();
}

}  // namespace odrive_can

#endif  // ODRIVE_CAN__MSG__DETAIL__O_DRIVE_STATUS__BUILDER_HPP_
