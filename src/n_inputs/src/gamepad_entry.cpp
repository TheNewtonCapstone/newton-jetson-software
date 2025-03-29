#include "rclcpp/rclcpp.hpp"

#include "nodes/gamepad_node.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    const auto gamepad_node = std::make_shared<newton::GamepadNode>();
    rclcpp::spin(gamepad_node);

    rclcpp::shutdown();

    return 0;
}