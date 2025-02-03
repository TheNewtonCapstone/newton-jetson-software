#include "motor.h"

#include <rclcpp/rclcpp.hpp>
#include "../../utils/include/result.h"

using namespace newton;
using namespace std::chrono_literals;

MotorDriver::MotorDriver(const rclcpp::NodeOptions& options)
    : Node("motor_driver") {
  // Declare parameters
  for (auto joint_name : joint_names) {
    this->declare_parameter(std::string("m_") + joint_name + "_node_id", "-1");
  }

  for (int i = 0; i < joint_names.size(); ++i) {
    auto node_id =
        this->get_parameter(std::string("m_") + joint_names[i] + "_node_id")
            .as_string();

    auto name = joint_names[i];

    status_subs[i] =
        this->create_subscription<odrive_can::msg::ODriveStatus>(
            name + "/odrive_status", 10,
            [=,this](const odrive_can::msg::ODriveStatus::SharedPtr msg) {
              this->update_driver_status(msg, i);
  });

  joint_state_subs[i] =
        this->create_subscription<odrive_can::msg::ControllerStatus>(
            name + "/controller_status", 10,
            [=,this](const odrive_can::msg::ControllerStatus::SharedPtr msg) {
              this->update_joint_state(msg, i);
  });

  control_pubs[i] =
  this->create_publisher<odrive_can::msg::ControlMessage>(
          name + "/control_message", 10);

    static float last_position = 0.0;
    auto timer_callback = [this](){
        for(int i = 0; i < 2; ++i){
          this->set_joint_position(last_position += 0.5, i);
        }
    };
    timer_ = this->create_wall_timer(1000ms, timer_callback);


    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", name.c_str());
  }
};

void MotorDriver::update_driver_status(
    const odrive_can::msg::ODriveStatus::SharedPtr msg, int joint_index) {

    auto bus_voltage = msg->bus_voltage;
    auto motor_temperature = msg->motor_temperature;
    auto bus_current = msg->bus_current;
    auto active_errors = msg->active_errors;
    auto disarm_reasons = msg->disarm_reason;
    auto fet_temperature = msg->fet_temperature;
  // Update joint state
  // RCLCPP_INFO(this->get_logger(),
  //             "Joint %s: bus_voltage=%f, motor_temperature=%f, bus_current=%f, "
  //             "active_errors=%u, disarm_reasons=%u, fet_temperature=%f",
  //             joint_names[joint_index].c_str(), bus_voltage, motor_temperature,
  //             bus_current, active_errors, disarm_reasons, fet_temperature);
}


void MotorDriver::update_joint_state(
    const odrive_can::msg::ControllerStatus::SharedPtr msg, int joint_index) {
        newton::joint::state new_state = 
          {
            .index = joint_index,
            .position = msg->pos_estimate,
           .velocity = msg->vel_estimate,
           .torque = msg->torque_estimate,
           .torque_target = msg->torque_estimate,
           };
        joint_states[joint_index] = new_state;

        RCLCPP_INFO(this->get_logger(),
          "Updated joint state for %d: position=%f, velocity=%f, torque=%f",
          joint_states[joint_index].index,
          joint_states[joint_index].position,
          joint_states[joint_index].velocity,
          joint_states[joint_index].torque);
    }

    void MotorDriver::set_joint_position(float position, int index) {
      odrive_can::msg::ControlMessage msg;
      msg.control_mode = 2; 
      msg.input_mode = 1;  
      msg.input_pos = 0; 
      msg.input_vel = 0.0;  
      msg.input_torque = 0.0; 
      control_pubs[index]->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Set joint position to %f for %d", position, index);
    }
