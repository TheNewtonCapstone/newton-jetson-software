#include "motor.h"

namespace newton {

Motor void Motor::statusCallback(
    const odrive_can::msg::ControllerStatus::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(status_mutex_);
  current_pos_ = msg->pos_estimate;
  current_vel_ = msg->vel_estimate;
  current_torque_ = msg->torque_estimate;
  axis_state_ = msg->axis_state;
  active_errors_ = msg->active_errors;
}

Motor::Motor(const std::string& name) : Node(name) {
  // Set up subscriber for position updates
  status_sub_ = create_subscription<odrive_can::msg::ControllerStatus>(
      "controller_status", 10,
      std::bind(&Motor::statusCallback, this, std::placeholders::_1));
}

}  // namespace newton