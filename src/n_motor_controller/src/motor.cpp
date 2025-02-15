#include "motor.h"
#include "logger.h"

#include <rclcpp/rclcpp.hpp>

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
  init();
};

result<void> MotorDriver::init() {
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
    msg_0.control_mode = 2;  
    msg_0.input_mode = 1;    
    msg_0.input_pos = 0.0;
    msg_0.input_vel = 5.0;
    msg_0.input_torque = 0.0;

    
    odrive_can::msg::ControlMessage msg_1;
    msg_1.control_mode = 2;  
    msg_1.input_mode = 1;    
    msg_1.input_pos = 0.0;
    msg_1.input_vel = 5.0;
    msg_1.input_torque = 0.0;

    control_pubs[0]->publish(msg_0);
    control_pubs[1]->publish(msg_1);
    return result<void>::success();
  }
void MotorDriver::update_driver_status(
    const odrive_can::msg::ODriveStatus::SharedPtr msg, int joint_index) {

    // auto bus_voltage = msg->bus_voltage;
    // auto motor_temperature = msg->motor_temperature;
    // auto bus_current = msg->bus_current;
    // auto active_errors = msg->active_errors;
    // auto disarm_reasons = msg->disarm_reason;
    // auto fet_temperature = msg->fet_temperature;
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



result<void> MotorDriver::request_axis_state(size_t joint_index, uint32_t requested_state) {
  auto client = this->clients[joint_index];
  const auto& joint_name= joint_names[joint_index];

  while(!client->wait_for_service(std::chrono::seconds(1))) {
    if(!rclcpp::ok()){
      return result<void>::error("MotorDriver", "Interrupted while waiting for service for joint %s", joint_name.c_str());
    }
    Logger::WARN("MotorDriver", "Waiting for axis state service for joint %s", joint_name.c_str());
  }

  auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
  request->axis_requested_state = requested_state;

  Logger::INFO("MotorDriver", "Requesting state %u for joint %s", requested_state, joint_name.c_str());
              
  auto future = client->async_send_request(request);
  auto timeout = std::chrono::seconds(10);
  auto future_status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout);
  
  
  // future status has 3 possible values: SUCCESS, TIMEOUT, INTERRUPTED
  if (future_status == rclcpp::FutureReturnCode::TIMEOUT){
    return result<void>::error("Motor Driver", "Timeout while waiting for response for joint %s", joint_name.c_str());
  } 
  
  if (future_status == rclcpp::FutureReturnCode::INTERRUPTED) {
    return result<void>::error("Motor Driver", "Interrupted while waiting for response for joint %s", joint_name.c_str());
  }
  
  
  if(future_status == rclcpp::FutureReturnCode::SUCCESS && future.get()->procedure_result != 0) {
      return result<void>::error("Motor Driver", "Failed to set state for joint %s", joint_name.c_str());
  } 
    
    return result<void>::success();
}

