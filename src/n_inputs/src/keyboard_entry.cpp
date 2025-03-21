#include "rclcpp/rclcpp.hpp"

#include "nodes/keyboard_node.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  const auto keyboard_node = std::make_shared<newton::KeyboardNode>();
  keyboard_node->run();

  rclcpp::shutdown();

  return 0;
}
