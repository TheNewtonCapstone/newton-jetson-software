// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from odrive_can:msg/ControlMessage.idl
// generated code does not contain a copyright notice

#ifndef ODRIVE_CAN__MSG__DETAIL__CONTROL_MESSAGE__BUILDER_HPP_
#define ODRIVE_CAN__MSG__DETAIL__CONTROL_MESSAGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "odrive_can/msg/detail/control_message__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace odrive_can
{

namespace msg
{

namespace builder
{

class Init_ControlMessage_input_torque
{
public:
  explicit Init_ControlMessage_input_torque(::odrive_can::msg::ControlMessage & msg)
  : msg_(msg)
  {}
  ::odrive_can::msg::ControlMessage input_torque(::odrive_can::msg::ControlMessage::_input_torque_type arg)
  {
    msg_.input_torque = std::move(arg);
    return std::move(msg_);
  }

private:
  ::odrive_can::msg::ControlMessage msg_;
};

class Init_ControlMessage_input_vel
{
public:
  explicit Init_ControlMessage_input_vel(::odrive_can::msg::ControlMessage & msg)
  : msg_(msg)
  {}
  Init_ControlMessage_input_torque input_vel(::odrive_can::msg::ControlMessage::_input_vel_type arg)
  {
    msg_.input_vel = std::move(arg);
    return Init_ControlMessage_input_torque(msg_);
  }

private:
  ::odrive_can::msg::ControlMessage msg_;
};

class Init_ControlMessage_input_pos
{
public:
  explicit Init_ControlMessage_input_pos(::odrive_can::msg::ControlMessage & msg)
  : msg_(msg)
  {}
  Init_ControlMessage_input_vel input_pos(::odrive_can::msg::ControlMessage::_input_pos_type arg)
  {
    msg_.input_pos = std::move(arg);
    return Init_ControlMessage_input_vel(msg_);
  }

private:
  ::odrive_can::msg::ControlMessage msg_;
};

class Init_ControlMessage_input_mode
{
public:
  explicit Init_ControlMessage_input_mode(::odrive_can::msg::ControlMessage & msg)
  : msg_(msg)
  {}
  Init_ControlMessage_input_pos input_mode(::odrive_can::msg::ControlMessage::_input_mode_type arg)
  {
    msg_.input_mode = std::move(arg);
    return Init_ControlMessage_input_pos(msg_);
  }

private:
  ::odrive_can::msg::ControlMessage msg_;
};

class Init_ControlMessage_control_mode
{
public:
  Init_ControlMessage_control_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ControlMessage_input_mode control_mode(::odrive_can::msg::ControlMessage::_control_mode_type arg)
  {
    msg_.control_mode = std::move(arg);
    return Init_ControlMessage_input_mode(msg_);
  }

private:
  ::odrive_can::msg::ControlMessage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::odrive_can::msg::ControlMessage>()
{
  return odrive_can::msg::builder::Init_ControlMessage_control_mode();
}

}  // namespace odrive_can

#endif  // ODRIVE_CAN__MSG__DETAIL__CONTROL_MESSAGE__BUILDER_HPP_
