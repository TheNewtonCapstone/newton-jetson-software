#include "nodes/gamepad_node.h"

using namespace newton;
using namespace std::chrono_literals;

GamepadNode::GamepadNode() : Node("gamepad_node") {
  m_joy_sub = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      [this](sensor_msgs::msg::Joy::SharedPtr msg) { joy_cb(msg); });
  m_twist_pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  m_switch_gait_pub =
      create_publisher<std_msgs::msg::String>("/change_gait", 10);
  m_switch_cmd_mode_pub =
      create_publisher<std_msgs::msg::Empty>("/preempt_teleop", 10);
}

void GamepadNode::publish_twist(const Vector3 &linear, const Vector3 &angular) {
  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = linear.x;
  twist_msg.linear.y = -linear.y;
  twist_msg.linear.z = linear.z;

  twist_msg.angular.x = angular.x;
  twist_msg.angular.y = angular.y;
  twist_msg.angular.z = angular.z;

  m_twist_pub->publish(twist_msg);
}

void newton::GamepadNode::joy_cb(const sensor_msgs::msg::Joy::SharedPtr msg) {
  // Check if the button is pressed
  if (msg->buttons[A]) {
    publish_jump();
  } else if (msg->buttons[X]) {
    publish_standing();
  } else if (msg->buttons[B]) {
    publish_machine();
  } else if (msg->buttons[Y]) {
    publish_switch_cmd_mode();
  }

  // Publish the twist message
  Vector3 linear(msg->axes[LEFT_STICK_X], msg->axes[LEFT_STICK_Y], 0.f);
  Vector3 angular(0.f, 0.f, msg->axes[RIGHT_STICK_X]);

  publish_twist(linear, angular);
}

void GamepadNode::publish_jump() { publish_gait_switch("jumping"); }
void GamepadNode::publish_standing() { publish_gait_switch("standing"); }
void GamepadNode::publish_machine() { publish_gait_switch("machine"); }

void GamepadNode::publish_switch_cmd_mode() {
  std_msgs::msg::Empty empty_msg;

  m_switch_cmd_mode_pub->publish(empty_msg);
}

void GamepadNode::publish_gait_switch(const std::string &gait_name) {
  std_msgs::msg::String msg;

  // gait is jumping
  msg.data = gait_name;
  m_switch_gait_pub->publish(msg);

  Logger::DEBUG("GamepadNode", "Published %s command", msg.data.c_str());
}
