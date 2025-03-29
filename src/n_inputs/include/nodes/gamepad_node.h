#pragma once

#include <chrono>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "gainput/gainput.h"
#include "result.h"
#include "data/vectors.h"

namespace newton
{
  enum Actions
  {
    HORIZONTAL,
    VERTICAL,
    ROTATE,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    TURN_LEFT,
    TURN_RIGHT,
  };

  class GamepadNode : public rclcpp::Node
  {
  public:
    GamepadNode();
    ~GamepadNode() = default;

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;

    std::unique_ptr<gainput::InputManager> m_input_manager;
    std::unique_ptr<gainput::InputMap> m_input_map;
    gainput::DeviceId m_gamepad_id;

    void read_gamepad();
    void publish_twist(const Vector3 &linear, const Vector3 &angular);
  };
} // namespace newton
