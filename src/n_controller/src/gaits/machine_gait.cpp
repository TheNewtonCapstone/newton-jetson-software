#include "gaits/machine_gait.h"
#include "logger.h"

#include <rclcpp/rclcpp.hpp>

using namespace newton;

MachineGait::MachineGait(const rclcpp::NodeOptions &options)
    : BaseGait("machine_gait", true, options)
{
  // Declare node's parameters default value
  // [NOT NECESSARY TO CHANGE THE FOLLOWING, USE ../config/params.yaml INSTEAD
  // !]
  this->declare_parameter("model_path", "model.onnx");
  this->declare_parameter("num_inputs", 33);
  this->declare_parameter("num_outputs", 8);

  // Get the parameters
  const auto model_path = this->get_parameter("model_path").as_string();
  const auto num_inputs = this->get_parameter("num_inputs").as_int();
  const auto num_outputs = this->get_parameter("num_outputs").as_int();

  onnx_handler =
      std::make_unique<OnnxHandler>(model_path, num_inputs, num_outputs);

  Logger::get_instance().set_logfile("machine_gait.log");

  Logger::INFO("machine_gait", 
    "time,angular_velocity_x,angular_velocity_y,angular_velocity_z,projected_gravity_x,projected_gravity_y,projected_gravity_z,linear_velocity_x,linear_velocity_y,angular_velocity_z,"
    "joint_delta_1,joint_delta_2,joint_delta_3,joint_delta_4,joint_delta_5,joint_delta_6,joint_delta_7,joint_delta_8,"
    "joint_velocity_1,joint_velocity_2,joint_velocity_3,joint_velocity_4,joint_velocity_5,joint_velocity_6,joint_velocity_7,joint_velocity_8"
    "previous_action_1,previous_action_2,previous_action_3,previous_action_4,previous_action_5,previous_action_6,previous_action_7,previous_action_8,"
    "action_1,action_2,action_3,action_4,action_5,action_6,action_7,action_8,");

  BaseGait::init();
}

result<void> MachineGait::move()
{
  std::string log_line = "";

  auto &input_buffer = onnx_handler->get_input_buffer();

  // angular velocity
  const auto angular_velocity_scaler = 0.25;
  input_buffer[0] = imu->angular_velocity.x * angular_velocity_scaler;
  input_buffer[1] = imu->angular_velocity.y * angular_velocity_scaler;
  input_buffer[2] = imu->angular_velocity.z * angular_velocity_scaler * -1.0;  // added -1.0 to invert the z axis

  // projected gravity
  input_buffer[3] = imu->projected_gravity.x;
  input_buffer[4] = imu->projected_gravity.y;
  input_buffer[5] = imu->projected_gravity.z;

  // commands
  input_buffer[6] = cmd->linear_velocity.x;
  input_buffer[7] = cmd->linear_velocity.y;
  input_buffer[8] = cmd->angular_velocity.z;

  // joint positions
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    input_buffer[9 + i] = joints[i].curr_pos - standing_positions[i];
  }

  // joint velocities
  const auto joint_velocity_scaler = 0.05;
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    input_buffer[17 + i] = joints[i].curr_vel * joint_velocity_scaler;
  }

  // previous actions
  const auto previous_actions_scaler = 1.0;
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    input_buffer[25 + i] = previous_actions[i] * previous_actions_scaler;
  }

  for (int i = 0; i < 33; i++)
  {
    log_line += std::to_string(input_buffer[i]) + ",";
  }

  onnx_handler->run();

  const auto &output_buffer = onnx_handler->get_output_buffer();

  std::array<float, NUM_JOINTS> positions{};
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    previous_actions[i] = output_buffer[i];
    float action_scaler = 0.25;
    auto delta = output_buffer[i] * action_scaler;
    positions[i] = standing_positions[i] + delta;

    log_line += std::to_string(positions[i]) + ((i < NUM_JOINTS -1) ? "," : "");
  }

  Logger::INFO("machine_gait", log_line.c_str());

  set_joints_position(positions);

  return result<void>::success();
}
