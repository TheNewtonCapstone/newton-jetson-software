#include <rclcpp/rclcpp.hpp>
#include "motor.h"
#include <iostream>

int main(int argc, char* argv[]) {
    std::cout << "Program starting..." << std::endl;
    
    try {
        rclcpp::init(argc, argv);
        auto logger = rclcpp::get_logger("motor_main");
        
        RCLCPP_INFO(logger, "ROS 2 initialized successfully");
        RCLCPP_INFO(logger, "Creating Motor node...");
        
        auto node = std::make_shared<newton::Motor>("1");
        RCLCPP_INFO(logger, "Motor node created, starting spin...");
        
        rclcpp::spin(node);
        RCLCPP_INFO(logger, "Spin completed");
        
        rclcpp::shutdown();
        RCLCPP_INFO(logger, "ROS 2 shut down successfully");
    } catch (const std::exception& e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
