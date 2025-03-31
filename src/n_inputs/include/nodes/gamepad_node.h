#pragma once

#include <chrono>

#include "data/vectors.h"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "result.h"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

namespace newton {
enum GamepadButton {
  A = 0,
  B = 1,
  X = 2,
  Y = 3,
  BACK = 4,
  GUIDE = 5,
  START = 6,
  LEFT_STICK = 7,
  RIGHT_STICK = 8,
  LEFT_SHOULDER = 9,
  RIGHT_SHOULDER = 10,
  DPAD_UP = 11,
  DPAD_DOWN = 12,
  DPAD_LEFT = 13,
  DPAD_RIGHT = 14,
  MISC_1 = 15,
  PADDLE_1 = 16,
  PADDLE_2 = 17,
  PADDLE_3 = 18,
  PADDLE_4 = 19,
  TOUCHPAD = 20,
};

enum GamepadAxis {
  LEFT_STICK_X = 0,
  LEFT_STICK_Y = 1,
  RIGHT_STICK_X = 2,
  RIGHT_STICK_Y = 3,
  LEFT_TRIGGER = 4,
  RIGHT_TRIGGER = 5,
};

class GamepadNode : public rclcpp::Node {
 public:
  GamepadNode();
  ~GamepadNode() = default;

 private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr m_joy_sub;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_twist_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_switch_gait_pub;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr m_switch_cmd_mode_pub;

  void joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg);

  void publish_jump();
  void publish_standing();
  void publish_machine();

  void publish_switch_cmd_mode();
  void publish_twist(const Vector3 &linear, const Vector3 &angular);
  void publish_gait_switch(const std::string &gait_name);
};
}  // namespace newton
