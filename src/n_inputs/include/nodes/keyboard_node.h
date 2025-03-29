#pragma once

#include <bits/types/struct_timeval.h>
#include <chrono>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace newton
{
  struct KeyPress
  {
    char key;
    uint64_t timestamp;

    bool operator<(const KeyPress &other) const
    {
      return timestamp < other.timestamp;
    };

    bool operator>(const KeyPress &other) const
    {
      return timestamp > other.timestamp;
    };

    bool operator==(const KeyPress &other) const { return key == other.key; };

    bool operator!=(const KeyPress &other) const { return !(*this == other); };
  };

  class KeyboardNode : public rclcpp::Node
  {
  public:
    KeyboardNode();
    ~KeyboardNode();

    void run();
    void publish_key_sequence(const std::unordered_map<char, uint64_t> &buffer);

  private:
    std::unordered_map<char, uint64_t> m_pressed_keys;
    struct termios m_old_tio, m_new_tio;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;

    inline bool key_available()
    {
      struct timeval timeout = {0, 0};
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(STDIN_FILENO, &read_fds);
      return select(STDIN_FILENO + 1, &read_fds, NULL, NULL, &timeout) > 0;
    };

    inline void set_non_blocking_mode()
    {
      tcgetattr(STDIN_FILENO, &m_old_tio);
      m_new_tio = m_old_tio;
      m_new_tio.c_lflag &= ~(ICANON | ECHO); // disable line buffering and echo
      tcsetattr(STDIN_FILENO, TCSANOW, &m_new_tio);
    };

    inline void reset_blocking_mode()
    {
      tcsetattr(STDIN_FILENO, TCSANOW, &m_old_tio);
    };
  };
} // namespace newton
