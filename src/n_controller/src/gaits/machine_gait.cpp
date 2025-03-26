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
  std::string log_title ="time,";
  log_title += "ang_vel_x, ang_vel_y, ang_vel_z,";
  log_title += "proj_grav_x, proj_grav_y, proj_grav_z,";
  log_title += "cmd_lin_vel_x, cmd_lin_vel_y, cmd_ang_vel_z,";
  log_title += "fl_hfe, fl_kfe, fr_hfe, fr_kfe, hl_hfe, hl_kfe, hr_hfe, hr_kfe,";
  log_title += "vel_fl_hfe, vel_fl_kfe, vel_fr_hfe, vel_fr_kfe, vel_hl_hfe, vel_hl_kfe, vel_hr_hfe, vel_hr_kfe,";
  log_title += "act_fl_hfe, act_fl_kfe, act_fr_hfe, act_fr_kfe, act_hl_hfe, act_hl_kfe, act_hr_hfe, act_hr_kfe,";
  Logger::INFO("machine_gait", log_title.c_str());

  BaseGait::init();
}

result<void> MachineGait::move()
{
  std::string log_line = "";
  auto now = this->get_clock()->now();
  log_line += std::to_string(now.nanoseconds()) + ",";

  auto &input_buffer = onnx_handler->get_input_buffer();


  // angular velocity
  const auto angular_velocity_scaler = 1.0; // default 0.25
  input_buffer[0] = imu->angular_velocity.x * angular_velocity_scaler;
  input_buffer[1] = imu->angular_velocity.y * angular_velocity_scaler;
  input_buffer[2] = imu->angular_velocity.z * angular_velocity_scaler;

  // projected gravity
  input_buffer[3] = imu->projected_gravity.x;
  input_buffer[4] = imu->projected_gravity.y;
  input_buffer[5] = imu->projected_gravity.z;

  // commands
  input_buffer[6] = cmd->linear_velocity.x;
  input_buffer[7] = cmd->linear_velocity.y;
  input_buffer[8] = cmd->angular_velocity.z;

  int position_idx = 9;
  int velocity_idx = 9 + NUM_JOINTS;

  int prev_action_idx = velocity_idx + NUM_JOINTS;

  int action_idx = prev_action_idx + NUM_JOINTS;
  // joint positions
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    input_buffer[position_idx + i] = joints[i].curr_pos - standing_positions[i];
  }

  // joint velocities
  const float joint_velocity_scaler = 0.05; // 0,.05
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    input_buffer[velocity_idx + i] = joints[i].curr_vel * joint_velocity_scaler;
  }

  // previous actions
  const float previous_actions_scaler = 1.0;
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    input_buffer[prev_action_idx + i] = previous_actions[i] * previous_actions_scaler;
  }

  for (int i = 0; i < 33; i++)
  {
    log_line += std::to_string(input_buffer[i]) + ",";
  }

  onnx_handler->run();

  const auto &output_buffer = onnx_handler->get_output_buffer();

  std::array<float, NUM_JOINTS> positions{};
  std::array<float, NUM_JOINTS> moded_positions{};

  for (int i = 0; i < NUM_JOINTS; i++)
  {
    previous_actions[i] = output_buffer[i];
    float action_scaler = 1.0; // default 0.25
    auto delta = output_buffer[i] * action_scaler;
    positions[i] = standing_positions[i] + delta;
    // moded_positions[i] = normalize_angle(positions[i]);
    log_line += std::to_string(positions[i]) + ((i < NUM_JOINTS -1) ? "," : "");
  }

  // for (int i = 0; i < NUM_JOINTS; i++)
  // {
  // log_line += std::to_string(moded_positions[i]) + ((i < NUM_JOINTS -1) ? ";" : "");
  // }


  Logger::INFO("machine_gait", log_line.c_str());

  set_joints_position(standing_positions);

  return result<void>::success();
}
