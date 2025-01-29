// motor_control/include/motor_control/motor.h
#pragma once

#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace newton {

class MotorDriver : public rclcpp::Node {
 public:
  explicit MotorDriver(const std::string& _id);

 private:
  void statusCallback(const odrive_can::msg::ControllerStatus::SharedPtr msg);
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr state_sub;

  std::string id;
  float position;
};

}  // namespace newton
