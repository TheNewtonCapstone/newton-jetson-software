#include <rclcpp/rclcpp.hpp>
#include "gaits/harmonic_gait.h"
#include "gaits/machine_gait.h"
#include "gaits/walking_gait.h"
#include "gaits/sliding_gait.h"
#include <iostream>

using namespace newton;
int main(int argc, char *argv[])
{
    std::cout << "Program starting..." << std::endl;

    try
    {
        rclcpp::init(argc, argv);
        auto logger = rclcpp::get_logger("controller_node");

        RCLCPP_INFO(logger, "ROS 2 initialized successfully");
        RCLCPP_INFO(logger, "Creating Controller node...");

        std::this_thread::sleep_for(std::chrono::seconds(5));
        // auto node = std::make_shared<newton::SlidingGait>();
        // auto node = std::make_shared<newton::MachineGait>();
        // auto node = std::make_shared<newton::HarmonicGait>();
        auto node = std::make_shared<newton::WalkingGait>();
        RCLCPP_INFO(logger, "Controller node created, starting spin...");

        rclcpp::spin(node);
        RCLCPP_INFO(logger, "Spin completed");

        rclcpp::shutdown();
        RCLCPP_INFO(logger, "ROS 2 shut down successfully");
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
