#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "gaits/gait_manager.h"
#include "gaits/machine_gait.h"

using namespace newton;

int main(int argc, char *argv[]) {
  std::cout << "Program starting..." << std::endl;

  try {
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("controller_node");

    RCLCPP_INFO(logger, "ROS 2 initialized successfully");
    RCLCPP_INFO(logger, "Creating Controller node...");

    std::this_thread::sleep_for(std::chrono::seconds(5));
    auto node = std::make_shared<newton::GaitManager>();
    RCLCPP_INFO(logger, "Controller node created, starting spin...");

    rclcpp::spin(node);
    RCLCPP_INFO(logger, "Spin completed");

    rclcpp::shutdown();
    RCLCPP_INFO(logger, "ROS 2 shut down successfully");
  } catch (const std::exception &e) {
    std::cerr << "Exception caught: " << e.what() << std::endl;
    return 1;
  }

  return 0;
}
