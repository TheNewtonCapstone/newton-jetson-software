#include "nodes/gamepad_node.h"
using namespace newton;

GamepadNode::GamepadNode() : Node("gamepad_node") {
  m_input_manager = std::make_unique<gainput::InputManager>();

  m_gamepad_id = m_input_manager->CreateDevice<gainput::InputDevicePad>();

  m_input_map = std::make_unique<gainput::InputMap>(*m_input_manager);
  m_input_map->MapFloat(HORIZONTAL, m_gamepad_id, gainput::PadButtonLeftStickX);
  m_input_map->MapFloat(VERTICAL, m_gamepad_id, gainput::PadButtonLeftStickY);
  m_input_map->MapFloat(ROTATE, m_gamepad_id, gainput::PadButtonRightStickX);
  m_input_map->MapFloat(JUMP, m_gamepad_id,
                        gainput::PadButtonA);  // A = X on a PS controller
  m_input_map->MapFloat(SWITCH_CMD_MODE, m_gamepad_id,
                        gainput::PadButtonX);  // X = Square on a PS controller

  m_twist_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  m_switch_gait_pub =
      create_publisher<std_msgs::msg::String>("/change_gait", 10);
  m_switch_cmd_mode_pub =
      create_publisher<std_msgs::msg::Empty>("/preempt_teleop", 10);
  m_timer = create_wall_timer(std::chrono::milliseconds(10),
                              [this]() { read_gamepad(); });
}

void newton::GamepadNode::read_gamepad() {
  m_input_manager->Update();

  // Twist commands
  const auto horizontal = m_input_map->GetFloat(HORIZONTAL);
  const auto vertical = m_input_map->GetFloat(VERTICAL);
  const auto rotate = m_input_map->GetFloat(ROTATE);

  auto linear = Vector3(vertical, horizontal, 0.0f);
  auto angular = Vector3(0.0f, 0.0f, rotate);

  publish_twist(linear, angular);

  // Jump command
  if (m_input_map->GetBoolIsNew(JUMP)) publish_jump();

  // Switch command mode
  if (m_input_map->GetBoolIsNew(SWITCH_CMD_MODE)) publish_switch_cmd_mode();
}

void newton::GamepadNode::publish_twist(const Vector3 &linear,
                                        const Vector3 &angular) {
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = linear.x;
  twist_msg.linear.y = -linear.y;
  twist_msg.linear.z = linear.z;

  twist_msg.angular.x = angular.x;
  twist_msg.angular.y = angular.y;
  twist_msg.angular.z = angular.z;

  m_twist_pub->publish(twist_msg);
}

void newton::GamepadNode::publish_jump() {
  std_msgs::msg::String msg;
  // gait is jumping
  msg.data = "machine";
  Logger::WARN("GamepadNode", "published %s command", msg.data.c_str());
  m_switch_gait_pub->publish(msg);

  RCLCPP_INFO(get_logger(), "Jump command published");
}

void newton::GamepadNode::publish_switch_cmd_mode() {
  std_msgs::msg::Empty empty_msg;

  m_switch_cmd_mode_pub->publish(empty_msg);

  RCLCPP_INFO(get_logger(), "Switch command mode published");
}
