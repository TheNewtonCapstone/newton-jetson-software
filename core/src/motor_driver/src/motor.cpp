#include "motor.h"

#include <rclcpp/rclcpp.hpp>
#include "../../utils/include/result.h"
#include <cmath>

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

  clients[i] = this->create_client<odrive_can::srv::AxisState>(
                name + "/request_axis_state"
  );

  control_pubs[i] =
  this->create_publisher<odrive_can::msg::ControlMessage>(
          name + "/control_message", 10);

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", name.c_str());
  } 

  timer_ = this->create_wall_timer(500ms, std::bind(&MotorDriver::collect_data_callback, this));

  init();
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
    }

    void MotorDriver::set_joint_position(float position, int index) {
      odrive_can::msg::ControlMessage msg;
      msg.control_mode = 3; 
      msg.input_mode = 1;  
      msg.input_pos = position; 
      msg.input_vel = 0.0;  
      msg.input_torque = 0.0; 
      control_pubs[index]->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Set joint position to %f for %d", position, index);
}


void MotorDriver::request_axis_state(size_t joint_index, uint32_t requested_state) {
  auto client = this->clients[joint_index];
  const auto& joint_name= joint_names[joint_index];

  while(!client->wait_for_service(std::chrono::seconds(1))) {
    if(!rclcpp::ok()){
      RCLCPP_ERROR(this->get_logger(),
        "ROS interrupted while waiting for axis state service for joint %s",
        joint_name.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Trying to request axis change for node %s",
    joint_name.c_str());
  }

  auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
  request->axis_requested_state = requested_state;

  RCLCPP_INFO(this->get_logger(), "Requesting state %u for joint %s", 
              requested_state, joint_names[joint_index].c_str());
              
  auto future = client->async_send_request(request);
  auto timeout = std::chrono::seconds(10);
  auto future_status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout);
  
  if (future_status == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    if (response->procedure_result == 0) {
      RCLCPP_INFO(this->get_logger(), 
        "Successfully set state for joint %s", joint_name.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), 
        "Failed to set state for joint %s. Error code: %d", 
        joint_name.c_str(), response->procedure_result);
    }
  } else if (future_status == rclcpp::FutureReturnCode::TIMEOUT) {
    RCLCPP_ERROR(this->get_logger(), 
      "Service call timed out for joint %s after %ld seconds", 
      joint_name.c_str(), timeout.count());
  } else {
    RCLCPP_ERROR(this->get_logger(), 
      "Failed to call axis state service for joint %s", 
      joint_name.c_str());
  }

}



void MotorDriver::init() {
  RCLCPP_INFO(this->get_logger(), "Initializing MotorDriver...");
  
  request_axis_state(0, 1);
  RCLCPP_INFO(this->get_logger(), "Set to idles %d", 0);
  request_axis_state(1, 1);
  RCLCPP_INFO(this->get_logger(), "Set %d to idle", 1);

  std::this_thread::sleep_for(std::chrono::seconds(1));

  // init 
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    odrive_can::msg::ControlMessage msg;
    msg.control_mode = 3;  
    msg.input_mode = 1;    
    msg.input_pos = 0.0;
    msg.input_vel = 0.0;
    msg.input_torque = 0.0;
    
    control_pubs[i]->publish(msg);
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  request_axis_state(0, 8);
  RCLCPP_INFO(this->get_logger(), "Set to 1 %d", 0);
  request_axis_state(1, 8);
  RCLCPP_INFO(this->get_logger(), "Set %d to idle", 1);
    odrive_can::msg::ControlMessage msg_0;
    msg_0.control_mode = 3;  
    msg_0.input_mode = 1;    
    msg_0.input_pos = 5 ;
    msg_0.input_vel = 0.0;
    msg_0.input_torque = 0.0;

    
    odrive_can::msg::ControlMessage msg_1;
    msg_1.control_mode = 3;  
    msg_1.input_mode = 1;    
    msg_1.input_pos = 5;
    msg_1.input_vel = 0.0;
    msg_1.input_torque = 0.0;

    //control_pubs[0]->publish(msg_0);
    //control_pubs[1]->publish(msg_1);
  }


void MotorDriver::collect_data_callback(){

  newton::joint::state hfe_state={
      .index = joint_states[0].index,
      .position=joint_states[0].position,
      .velocity = joint_states[0].velocity,
      .torque = joint_states[0].torque,
      .torque_target = joint_states[0].torque_target,
    };

  newton::joint::state kfe_state = {
      .index = joint_states[1].index,
      .position = joint_states[1].position,
      .velocity = joint_states[1].velocity,
      .torque = joint_states[1].torque,
      .torque_target = joint_states[1].torque_target,
    };


  RCLCPP_INFO(this->get_logger(), "Call pack %f position", hfe_state.position);
  float  amplitude = 226.0f;
  float amplitude_sqr = amplitude * amplitude;
  int arm_length = 160; // mm 
  float arm_length_sqr = (float)arm_length * arm_length;
  float c = std::acos((amplitude_sqr - 2.f * arm_length_sqr)/(3.f * arm_length_sqr));

  float turns = c / 3.14159f;

  odrive_can::msg::ControlMessage msg_0;
  msg_0.control_mode = 3;  
  msg_0.input_mode = 1;    
  msg_0.input_pos = turns;
  msg_0.input_vel = 0.0;
  msg_0.input_torque = 0.0;

  // control_pubs[1]->publish(msg_0);
  
  RCLCPP_INFO(this->get_logger(), "Callback c: %f", c);
}
