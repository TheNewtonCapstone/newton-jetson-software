// motor_control/include/motor_control/motor.h
#pragma once

#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace newton {

class Motor : public rclcpp::Node {
 public:
  explicit Motor(const std::string& name);

  bool setPosition(float position, float velocity = 0, float torque = 0);
  bool setVelocity(float velocity, float torque = 0);
  bool setTorque(float torque);
  bool setState(uint32_t state);
  bool clearErrors();

  float getPosition() const { return current_pos_; }
  float getVelocity() const { return current_vel_; }
  float getTorque() const { return current_torque_; }
  uint32_t getState() const { return axis_state_; }
  uint32_t getErrors() const { return active_errors_; }

 private:
  void statusCallback(const odrive_can::msg::ControllerStatus::SharedPtr msg);

  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr cmd_pub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr state_sub;
  rclcpp::Subscription<odrive_can::msg::ODriveStatus>::SharedPtr state_sub;

  rlcpp::sub rclcpp::Client<odrive_can::srv::AxisState>::SharedPtr
      state_client_;

  float current_pos_{0};
  float current_vel_{0};
  float current_torque_{0};
  uint32_t axis_state_{0};
  uint32_t active_errors_{0};
  float bus_voltage { 0.0 }
};

}  // namespace newton