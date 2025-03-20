#include "gaits/machine_gait.h"
#include "logger.h"

#include <rclcpp/rclcpp.hpp>

using namespace newton;
using namespace std::chrono_literals;

MachineGait::MachineGait(const rclcpp::NodeOptions &options)
    : BaseGait("machine_gait", options) {
  // Declare node's parameters default value
  // [NOT NECESSARY TO CHANGE THE FOLLOWING, USE ../config/params.yaml INSTEAD
  // !]
  this->declare_parameter("model_path", "model.onnx");
  this->declare_parameter("num_inputs", 45);
  this->declare_parameter("num_outputs", 12);

  // Get the parameters
  const auto model_path = this->get_parameter("model_path").as_string();
  const auto num_inputs = this->get_parameter("num_inputs").as_int();
  const auto num_outputs = this->get_parameter("num_outputs").as_int();

  onnx_handler =
      std::make_unique<OnnxHandler>(model_path, num_inputs, num_outputs);

  BaseGait::init();
};

void MachineGait::move() {
  Logger::INFO("MachineGait", "Waiting for encoder offsets to be loaded");
  while (!offset_loaded) {
  };

  std::array<float, NUM_JOINTS> jp_mult = {
      1.0, 1.0,  -1.0, // fl
      1.0, -1.0, 1.0,  // fr
      1.0, 1.0,  -1.0, // hl
      1.0, -1.0, 1.0,  // hr
  };

  std::array<float, NUM_JOINTS> standing_positions = {
      0.0, 0.8, -1.5, // fl
      0.0, 0.8, -1.5, // fr
      0.0, 1.0, -1.5, // hl
      0.0, 1.0, -1.5, // hr
  };

  auto &input_buffer = onnx_handler->get_input_buffer();

  // angular velocity
  const auto angular_velocity_scaler = 0.25;
  input_buffer[0] = imu->angular_velocity.x * angular_velocity_scaler;
  input_buffer[1] = imu->angular_velocity.y * angular_velocity_scaler;
  input_buffer[2] = imu->angular_velocity.z * angular_velocity_scaler;

  // projected gravity
  input_buffer[3] = imu->projected_gravity.x;
  input_buffer[4] = imu->projected_gravity.y;
  input_buffer[5] = imu->projected_gravity.z;

  // commands
  input_buffer[6] = 0.0;
  input_buffer[7] = 0.0;
  input_buffer[8] = 0.0;

  // joint positions
  for (int i = 0; i < NUM_JOINTS; i++) {
    input_buffer[9 + i] = joints[i].cur_pos - standing_positions[i];
  }

  // joint velocities
  const auto joint_velocity_scaler = 0.05;
  for (int i = 0; i < NUM_JOINTS; i++) {
    input_buffer[21 + i] = joints[i].curr_vel * joint_velocity_scaler;
  }

  // previous actions
  const auto previous_actions_scaler = 1.0;
  for (int i = 0; i < NUM_JOINTS; i++) {
    input_buffer[33 + i] = previous_actions[i] * previous_actions_scaler;
  }

  onnx_handler->run();

  const auto &output_buffer = onnx_handler->get_output_buffer();

  std::array<double, NUM_JOINTS> positions{};
  for (int i = 0; i < NUM_JOINTS; i++) {
    previous_actions[i] = output_buffer[i];
    positions[i] = standing_positions[i] + (output_buffer[i] * jp_mult[i]);
  }

  // print the positions
  for (int i = 0; i < NUM_JOINTS; i++) {
    RCLCPP_INFO(this->get_logger(), "Joint %d: %f", i, positions[i]);
  }

  for (int i = 0; i < NUM_JOINTS; i++) {
    set_joint_position(standing_positions[i], i);
  }
}
