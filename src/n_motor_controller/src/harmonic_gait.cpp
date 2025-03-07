#include "harmonic_gait.h"

#include "logger.h"

#include <rclcpp/rclcpp.hpp>

using namespace newton;
using namespace std::chrono_literals;


HarmonicGait::HarmonicGait(const rclcpp::NodeOptions& options)
    : Node("motor_driver") {

  // Declare parameters
  for (auto joint_name : joint_names) {
    this->declare_parameter(std::string("m_") + joint_name + "_node_id", "-1");
  }

  init_clients();

};

result<void> HarmonicGait::init() {
  RCLCPP_INFO(this->get_logger(), "Initializing HarmonicGait...");
  
  request_axis_state(0, 1);
  request_axis_state(1, 1);
  request_axis_state(2, 1);

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
  auto res= request_axis_state(0, 8);

  auto res_1 = request_axis_state(1, 8);
  auto res_2 = request_axis_state(2, 8);

  if(res.has_error() || res_1.has_error() || res_2.has_error()){
    Logger::ERROR("Harmonic Gait", "Issues with init sequence exiting");
    std::exit(1);
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  /**
    odrive_can::msg::ControlMessage msg;
    msg.control_mode = 3;  
    msg.input_mode = 1;    
    msg.input_pos = 2;
    msg.input_vel = 0.0;
    msg.input_torque = 0.0;


    control_pubs[0]->publish(msg);
    control_pubs[1]->publish(msg);
    control_pubs[2]->publish(msg);
    
    last_time = this->get_clock()->now();

    */

    return result<void>::success();
}


result<void> HarmonicGait::init_clients(){
  for (int i = 0; i < joint_names.size(); ++i) {
    auto node_id =
        this->get_parameter(std::string("m_") + joint_names[i] + "_node_id")
            .as_string();

  auto name = joint_names[i];

  clients[i] = this->create_client<odrive_can::srv::AxisState>(
                name + "/request_axis_state"
  );
  control_pubs[i] =
  this->create_publisher<odrive_can::msg::ControlMessage>(
          name + "/control_message", 10);
  } 

  request_axis_state(0, 1);
  request_axis_state(1, 1);
  request_axis_state(2, 1);

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
  auto res= request_axis_state(0, 8);

  auto res_1 = request_axis_state(1, 8);
  auto res_2 = request_axis_state(2, 8);

  if(res.has_error() || res_1.has_error() || res_2.has_error()){
    Logger::ERROR("Harmonic Gait", "Issues with init sequence exiting");
    std::exit(1);
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  /**
    odrive_can::msg::ControlMessage msg;
    msg.control_mode = 3;  
    msg.input_mode = 1;    
    msg.input_pos = 2;
    msg.input_vel = 0.0;
    msg.input_torque = 0.0;


    control_pubs[0]->publish(msg);
    control_pubs[1]->publish(msg);
    control_pubs[2]->publish(msg);
    
    last_time = this->get_clock()->now();

    */

    return result<void>::success();
}
result<void> HarmonicGait::init_pubs(){
  for (int i = 0; i < joint_names.size(); ++i) {
    auto node_id =
        this->get_parameter(std::string("m_") + joint_names[i] + "_node_id")
            .as_string();

  auto name = joint_names[i];


  control_pubs[i] =
  this->create_publisher<odrive_can::msg::ControlMessage>(
          name + "/control_message", 10);

  RCLCPP_INFO(this->get_logger(), "Subscribed to %s", name.c_str());
  } 

  return result<void>::success();
}

result<void> HarmonicGait::init_subs(){

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
  
  

  } 
  return result<void>::success();
}

void HarmonicGait::update_driver_status(
    const odrive_can::msg::ODriveStatus::SharedPtr msg, int joint_index) {
}


void HarmonicGait::update_joint_state(
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

void HarmonicGait::set_joint_position(float position, int index) {
      odrive_can::msg::ControlMessage msg;
      msg.control_mode = 3; 
      msg.input_mode = 1;  
      msg.input_pos = position; 
      msg.input_vel = 0.0;  
      msg.input_torque = 0.0; 
      control_pubs[index]->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Set joint position to %f for %d", position, index);
}

void HarmonicGait::set_all_joint_positions(const std::array<float, 12> &positions) {
      for (int i = 0; i < NUM_JOINTS; i++) {
        if (!control_pubs[i]) {
        RCLCPP_WARN
          (this->get_logger(), "Control publisher for joint %d is not initialized", i);
          continue;
      }

        odrive_can::msg::ControlMessage msg;
        msg.control_mode = 2; 
        msg.input_mode = 1;  
        msg.input_pos = positions[i]; 
        msg.input_vel = 0.0;  
        msg.input_torque = 0.0; 
        control_pubs[i]->publish(msg);
      }
}

result<void> HarmonicGait::request_axis_state(size_t joint_index, uint32_t requested_state) {
  auto client = this->clients[joint_index];
  const auto& joint_name= joint_names[joint_index];

  while(!client->wait_for_service(std::chrono::seconds(1))) {
    if(!rclcpp::ok()){
      return result<void>::error("HarmonicGait", "Interrupted while waiting for service for joint %s", joint_name.c_str());
    }
    Logger::WARN("HarmonicGait", "Waiting for axis state service for joint %s", joint_name.c_str());
  }

  auto request = std::make_shared<odrive_can::srv::AxisState::Request>();
  request->axis_requested_state = requested_state;

  Logger::INFO("HarmonicGait", "Requesting state %u for joint %s", requested_state, joint_name.c_str());
              
  auto future = client->async_send_request(request);
  auto timeout = std::chrono::seconds(10);
  auto future_status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), future, timeout);
  
  
  // future status has 3 possible values: SUCCESS, TIMEOUT, INTERRUPTED
  if (future_status == rclcpp::FutureReturnCode::TIMEOUT){
    return result<void>::error("Motor Driver", "Timeout while waiting for response for joint %s, request timedout", joint_name.c_str());
  } 
  
  if (future_status == rclcpp::FutureReturnCode::INTERRUPTED) {
    return result<void>::error("Motor Driver", "Interrupted while waiting for response for joint %s", joint_name.c_str());
  }
  
  
  if(future_status == rclcpp::FutureReturnCode::SUCCESS && future.get()->procedure_result != 0) {
      return result<void>::error("Motor Driver", "Failed to set state for joint %s", joint_name.c_str());
  } 
    
  Logger::WARN("HarmonicGait", "Success");
    return result<void>::success();
}

void HarmonicGait::move() {
  const float TWO_PI = 6.28318530718;
  const float PI = 3.14159265359;
  const float link_length = 0.16;

  // determine the angle base on b
  const std::array <float, 14> b = {
    0.1, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.2, 0.21, 0.22, 0.23, 0.24
  };

  for(int i = 0; i < 14; i++) {
    float angle = acos((b[i] * b[i]) / (2.0f * link_length * b[i]));
    float angle_deg = acos(b[i]);
    
    RCLCPP_INFO(this->get_logger(), "Link %f\tAngle %f\t", angle, b[i], angle_deg);
  }

  auto now = this->get_clock()->now();
  

    float amplitude = 0.5;
    float frequency = 0.5;
    float phase = 0.0;

    float position = amplitude * sin(TWO_PI * frequency * now.nanoseconds() + phase);


  RCLCPP_INFO(this->get_logger(), "Harmonic motion %ld, Now", now.nanoseconds()); 
last_time = now;
}
