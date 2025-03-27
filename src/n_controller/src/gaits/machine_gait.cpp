#include "gaits/machine_gait.h"
#include "logger.h"

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sstream>
#include <vector>

using namespace newton;

MachineGait::MachineGait(const rclcpp::NodeOptions &options)
    : BaseGait("machine_gait", true, options)
{
  // Declare node's parameters default value
  this->declare_parameter("model_path", "model.onnx");
  this->declare_parameter("num_inputs", 33);
  this->declare_parameter("num_outputs", 8);
  this->declare_parameter("csv_path", "joint_positions.csv"); // New parameter for CSV file path

  // Get the parameters
  const auto model_path = this->get_parameter("model_path").as_string();
  const auto num_inputs = this->get_parameter("num_inputs").as_int();
  const auto num_outputs = this->get_parameter("num_outputs").as_int();
  const auto csv_path = this->get_parameter("csv_path").as_string(); // Get CSV path

  onnx_handler =
      std::make_unique<OnnxHandler>(model_path, num_inputs, num_outputs);

  // Open CSV file
  csv_file.open(csv_path);
  if (!csv_file.is_open()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_path.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Successfully opened CSV file: %s", csv_path.c_str());
    
    // Read and skip header line if it exists
    std::string header;
    std::getline(csv_file, header);
  }

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

// Helper function to parse CSV line into array of floats
std::array<float, 8> parseCsvLine(const std::string& line) {
  
  std::array<float, 8> values{};  // Initialize with zeros
  std::stringstream ss(line);
  std::string token;
  size_t index = 0;
  
  while (std::getline(ss, token, ',') && index < 8) {
    try {
      values[index++] = std::stof(token);
    } catch (const std::exception& e) {
      // Handle invalid values, keep as 0.0f
      index++;
    }
  }
  
  return values;
}

result<void> MachineGait::move()
{
  // Try to read the next line from CSV if file is open
  if (csv_file.is_open() && !csv_file.eof()) {
    std::string line;
    if (std::getline(csv_file, line)) {
      // Parse the CSV line
      csv_data = parseCsvLine(line);
      
    } else {
      // reset the file pointer to the beginning of the file
      csv_file.clear();
      csv_file.seekg(0, std::ios::beg);
    }
  }

  std::string log_line = "";
  auto now = this->get_clock()->now();
  log_line += std::to_string(now.seconds()) + ",";

  auto &input_buffer = onnx_handler->get_input_buffer();

  // angular velocity
  const auto angular_velocity_scaler = 0.25; // default 0.25
  input_buffer[0] = imu->angular_velocity.x * angular_velocity_scaler;
  input_buffer[1] = imu->angular_velocity.y * angular_velocity_scaler;
  input_buffer[2] = imu->angular_velocity.z * angular_velocity_scaler;

  // projected gravity
  input_buffer[3] = imu->projected_gravity.x;
  input_buffer[4] = imu->projected_gravity.y;
  input_buffer[5] = imu->projected_gravity.z;

  // commands
  input_buffer[6] = 1.0f; //cmd->linear_velocity.x;
  input_buffer[7] = cmd->linear_velocity.y;
  input_buffer[8] = cmd->angular_velocity.z;

  constexpr int POSITION_IDX = 9;
  constexpr int VELOCITY_IDX = POSITION_IDX + NUM_JOINTS;
  constexpr int PREV_ACTION_IDX = VELOCITY_IDX + NUM_JOINTS;
  constexpr int ACTION_IDX = PREV_ACTION_IDX + NUM_JOINTS;

  // joint positions
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    input_buffer[POSITION_IDX + i] = joints[i].curr_pos - standing_positions[i];
  }

  // joint velocities
  const float joint_velocity_scaler = 0.05; // 0.05
  for (int i = 0; i < NUM_JOINTS; i++) {
    input_buffer[VELOCITY_IDX + i] = joints[i].curr_vel * joint_velocity_scaler;
  }

  // previous actions
  const float previous_actions_scaler = 1.0;
  for (int i = 0; i < NUM_JOINTS; i++) {
    input_buffer[PREV_ACTION_IDX + i] = previous_actions[i] * previous_actions_scaler;
  }

  for (int i = 0; i < 33; i++) {
    log_line += std::to_string(input_buffer[i]) + ",";
  }

  onnx_handler->run();

  const auto &output_buffer = onnx_handler->get_output_buffer();

  std::array<float, NUM_JOINTS> positions{};

  for (int i = 0; i < NUM_JOINTS; i++) {
    previous_actions[i] = output_buffer[i];
    
    float action_scaler = 0.25; // default 0.25
    const auto delta = output_buffer[i] * action_scaler;
    positions[i] = standing_positions[i] + delta;

    log_line += std::to_string(positions[i]) + ((i < NUM_JOINTS -1) ? "," : "");
  }

  Logger::INFO("machine_gait_moving", log_line.c_str());

  set_joints_position(positions);

  return result<void>::success();
}