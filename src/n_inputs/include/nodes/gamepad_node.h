#pragma once

#include <chrono>

#include "data/vectors.h"
#include "gainput/gainput.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "result.h"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace newton {
enum Actions {
  HORIZONTAL,
  VERTICAL,
  ROTATE,
  JUMP,
  SWITCH_CMD_MODE,
};

class GamepadNode : public rclcpp::Node {
 public:
  GamepadNode();
  ~GamepadNode() = default;

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_twist_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_switch_gait_pub;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_switch_cmd_mode_pub;
  rclcpp::TimerBase::SharedPtr m_timer;

  std::unique_ptr<gainput::InputManager> m_input_manager;
  std::unique_ptr<gainput::InputMap> m_input_map;
  gainput::DeviceId m_gamepad_id;

  void read_gamepad();

  void publish_twist(const Vector3 &linear, const Vector3 &angular);
  void publish_jump();
  void publish_switch_cmd_mode();
};
}  // namespace newton
