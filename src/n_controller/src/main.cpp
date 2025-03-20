#include <rclcpp/rclcpp.hpp>
#include "gaits/harmonic_gait.h"
#include "gaits/machine_gait.h"
#include <iostream>

int main(int argc, char *argv[])
{
    std::cout << "Program starting..." << std::endl;

    try
    {
        rclcpp::init(argc, argv);
        auto logger = rclcpp::get_logger("motor_main");

        RCLCPP_INFO(logger, "ROS 2 initialized successfully");
        RCLCPP_INFO(logger, "Creating MotorDriver node...");

        std::this_thread::sleep_for(std::chrono::seconds(5));
        // auto node = std::make_shared<newton::HarmonicGait>();
        auto node = std::make_shared<newton::MachineGait>();
        RCLCPP_INFO(logger, "MotorDriver node created, starting spin...");

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
