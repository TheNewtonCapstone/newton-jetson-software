#include "nodes/keyboard_node.h"
#include "logger.h"
#include <chrono>
#include <unistd.h>

using namespace newton;
using namespace std::chrono_literals;

KeyboardNode::KeyboardNode() : Node("keyboard_node")
{
  m_publisher =
      this->create_publisher<geometry_msgs::msg::Twist>("/vel_cmds/keyboard", 10);

  set_non_blocking_mode();
}

KeyboardNode::~KeyboardNode() { reset_blocking_mode(); }

void KeyboardNode::run()
{
  constexpr auto KEY_TIMEOUT = (100ms).count();

  char key;
  uint64_t timestamp;

  while (rclcpp::ok())
  {
    timestamp = std::chrono::system_clock::now().time_since_epoch().count();

    // removes keys that have timed out
    for (auto it = m_pressed_keys.begin(); it != m_pressed_keys.end();)
    {
      auto it_timestamp = it->second;

      if (timestamp - it_timestamp > KEY_TIMEOUT)
      {
        it = m_pressed_keys.erase(it);
      }
      else
      {
        ++it;
      }
    }

    if (key_available())
    {
      read(STDIN_FILENO, &key, 1);

      if (key == 'q')
      {
        Logger::INFO("KeyboardNode", "Exiting...");
        break;
      }

      m_pressed_keys[key] = timestamp;

      publish_key_sequence(m_pressed_keys);
    }

    rclcpp::spin_some(this->get_node_base_interface());
  }

  reset_blocking_mode();
}

void KeyboardNode::publish_key_sequence(
    const std::unordered_map<char, uint64_t> &key_presses)
{
  if (key_presses.empty())
    return;

  auto msg = geometry_msgs::msg::Twist();

  for (const auto &key_press : key_presses)
  {
    auto key = key_press.first;

    switch (key)
    {
    case 'w':
      msg.linear.y += 1.0;
      break;
    case 's':
      msg.linear.y += -1.0;
      break;
    case 'a':
      msg.linear.x += -1.0;
      break;
    case 'd':
      msg.linear.x += 1.0;
      break;
    default:
      break;
    }
  }

  msg.linear.x = std::clamp(msg.linear.x, -1.0, 1.0);
  msg.linear.y = std::clamp(msg.linear.y, -1.0, 1.0);

  m_publisher->publish(msg);
}
