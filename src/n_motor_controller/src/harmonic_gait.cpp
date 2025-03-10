#include "harmonic_gait.h"

#include "logger.h"

#include <rclcpp/rclcpp.hpp>

using namespace newton;
using namespace std::chrono_literals;


HarmonicGait::HarmonicGait(const rclcpp::NodeOptions& options)
    : Node("motor_driver") {
  init_pubs();
  init_clients();
  init_subs();

  timer_ = this->create_wall_timer(20ms, std::bind(&HarmonicGait::move, this));
  

};

result<void> HarmonicGait::init_clients(){

  for (int i = 0; i < NUM_JOINTS; ++i) {
    auto name = joint_names[i];

    clients[i] = this->create_client<odrive_can::srv::AxisState>(
                name + "/request_axis_state"
  );
  }

  for (size_t i = 0; i < NUM_JOINTS; i++){
    if(disarm(i).has_error()){
      Logger::ERROR("Harmonic Gait", "Issues with init sequence exiting");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      std::exit(1);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Initializing HarmonicGait...");
  
  for (size_t i = 0; i < NUM_JOINTS; i++) {
    odrive_can::msg::ControlMessage msg;
    msg.control_mode = 3;  
    msg.input_mode = 5;    
    msg.input_pos = 0.0;
    msg.input_vel = 0.0;
    msg.input_torque = 0.0;
    control_pubs[i]->publish(msg);
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  for (size_t i = 0; i < NUM_JOINTS; i++){
    if(arm(i).has_error()){
      Logger::ERROR("Harmonic Gait", "Issues with init sequence exiting");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      std::exit(1);
    }
  }
  return result<void>::success();
}
result<void> HarmonicGait::init_pubs(){
  for (int i = 0; i < joint_names.size(); ++i) {
  auto name = joint_names[i];

  control_pubs[i] =
  this->create_publisher<odrive_can::msg::ControlMessage>(name + "/control_message", 10);

  RCLCPP_INFO(this->get_logger(), "Created a publisher to to %s", name.c_str());
  } 

  return result<void>::success();
}

result<void> HarmonicGait::init_subs(){

  for (int i = 0; i < NUM_JOINTS; ++i) {
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

        if(offset_loaded == false){
          encoder_offsets[joint_index] = msg->pos_estimate;
        }

        // first time we get to the end of the loop, we have all the offsets
        if(joint_index == NUM_JOINTS - 1){
          offset_loaded = true;
        }

        // RCLCPP_INFO(this->get_logger(),
        //   "Updated joint state for %d: position=%f, velocity=%f, torque=%f",
        //   joint_states[joint_index].index,
        //   joint_states[joint_index].position,
        //   joint_states[joint_index].velocity,
        //   joint_states[joint_index].torque);

}

void HarmonicGait::set_joint_position(float position, int index) {
      odrive_can::msg::ControlMessage msg;
      msg.control_mode = 3; 
      msg.input_mode = 1;  
      msg.input_pos = position + encoder_offsets[index]; 
      msg.input_vel = 0.0;  
      msg.input_torque = 0.0; 
      control_pubs[index]->publish(msg);
      RCLCPP_INFO(this->get_logger(), "Set joint position to %f for %d", position, index);
}

void HarmonicGait::set_all_joint_positions(const std::array<float, 12> &positions) {
      for (int i = 0; i < NUM_JOINTS; i++) {
        set_joint_position(positions[i], i);
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
    
  Logger::INFO("HarmonicGait", "Success");
    return result<void>::success();
}

void HarmonicGait::move() {

  const float DURATION_TIME = 60.0;
  const float amplitude = 0.5;
  const float frequency = 0.5;

  const float TWO_PI = 6.28318530718;
  const float PI = 3.14159265359;
  const float link_length = 0.16;

  // while(!offset_loaded){};
  
  auto now = this->get_clock()->now();

  if((this->get_clock()->now() - last_time).seconds() > DURATION_TIME){
    RCLCPP_INFO(this->get_logger(), "Finished Harmonic Gait");
    shutdown();
  }
  std::array<float, 12> offset = {
    0,0,0,
    0,0,0,
    0,0,0,
    0,0,0
  };

  const std::array <float, 12> standing_positions = {
    0.0 , 1, -2.0,
    0.0, - 4, 2.0, // fr    
    0.0,  - 1, -2.0, // hl
    0.0, 4, 2.0
  }; 


  
  // get the offsets 
  float base_position = amplitude * sin(TWO_PI * frequency * now.nanoseconds());
  float hfe_offset =  amplitude * - 1.0  * base_position;
  float kfe_offset =  amplitude * 2.0 * base_position;


  std::array<float, 12> positions;
  for(int i = 0; i < 12; i++){
    if(i % 3 == 1){
    positions[i] = standing_positions[i] + hfe_offset;
    } else if(i % 3 == 2){
    positions[i] = standing_positions[i] + kfe_offset;
  }
  }


  RCLCPP_INFO(this->get_logger(), "Harmonic motion %ld, Now", now.nanoseconds()); 
  // print the positions
  for(int i = 0; i < 12; i++){
    RCLCPP_INFO(this->get_logger(), "Joint %d: %f", i, positions[i]);
  }
  // set_all_joint_positions(positions);



last_time = now;
}


result<void> HarmonicGait::arm(int joint_index){
  auto res = request_axis_state(joint_index, 8);
  if(res.has_error()){
    return result<void>::error("Harmonic Gait", "Issues with init sequence exiting");
  }

  return result<void>::success();
}

result<void> HarmonicGait::disarm(int joint_index){
  auto res = request_axis_state(joint_index, 1);
  if(res.has_error()){
    return result<void>::error("Harmonic Gait", "Issues with init sequence exiting");
  }
  return result<void>::success();
}

void HarmonicGait::shutdown(){
  for (size_t i = 0; i < NUM_JOINTS; i++){
    disarm(i);
  }
  rclcpp::shutdown();
}




