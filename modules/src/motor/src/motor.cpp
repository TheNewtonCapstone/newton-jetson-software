#include "motor.h"
#include <rclcpp/rclcpp.hpp>


using namespace newton;
MotorDriver::MotorDriver(const std::string& _id)
    :Node("odrive_node_" + _id), position(0.0){
    // Declare parameters
    for(auto joint_name : newton::joint::names){
        this -> declare_parameter(std::string("m_") + joint_name + "_node_id", "00");
    }

    // Create timer
    auto timer_callback = [this](){
        for(auto joint_name : newton::joint::names){
            auto params = this->get_parameter(std::string("m_") + joint_name + "_node_id");
            RCLCPP_INFO(this->get_logger(), "%s: %s", joint_name.c_str(), params.as_string().c_str());
        }
    };

    timer_ = this->create_wall_timer(1000ms, timer_callback);

    
};

void MotorDriver::statusCallback(const odrive_can::msg::ControllerStatus::SharedPtr msg) {
    printf("Ping\n");
}


