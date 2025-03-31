#include "gaits/machine_gait.h"

#include <fstream>
#include <sstream>
#include <vector>

#include "logger.h"

using namespace newton;

MachineGait::MachineGait(const std::string& model_path) : BaseGait() {
  onnx_handler =
      std::make_unique<OnnxHandler>(model_path, 33, 8);

  // Open CSV file
  const std::string csv_path = "joint_positions.csv";
  csv_file.open(csv_path);
  if (!csv_file.is_open()) {
    Logger::ERROR("machine_gait",
                 "Failed to open CSV file: %s", csv_path.c_str());
  } else {
    Logger::INFO("machine_gait",
                "Successfully opened CSV file: %s", csv_path.c_str());

    // Read and skip header line if it exists
    std::string header;
    std::getline(csv_file, header);
  }

  Logger::get_instance().set_logfile("machine_gait.csv");
  std::string log_title = "time,";
  log_title += "ang_vel_x, ang_vel_y, ang_vel_z,";
  log_title += "proj_grav_x, proj_grav_y, proj_grav_z,";
  log_title += "cmd_lin_vel_x, cmd_lin_vel_y, cmd_ang_vel_z,";
  log_title +=
      "fl_hfe, fl_kfe, fr_hfe, fr_kfe, hl_hfe, hl_kfe, hr_hfe, hr_kfe,";
  log_title +=
      "vel_fl_hfe, vel_fl_kfe, vel_fr_hfe, vel_fr_kfe, vel_hl_hfe, vel_hl_kfe, "
      "vel_hr_hfe, vel_hr_kfe,";
  log_title +=
      "act_fl_hfe, act_fl_kfe, act_fr_hfe, act_fr_kfe, act_hl_hfe, act_hl_kfe, "
      "act_hr_hfe, act_hr_kfe,";
  Logger::INFO("machine_gait", log_title.c_str());
}

// Helper function to parse CSV line into array of floats
std::array<float, 8> parseCsvLine(const std::string &line) {
  std::array<float, 8> values{};  // Initialize with zeros
  std::stringstream ss(line);
  std::string token;
  size_t index = 0;

  while (std::getline(ss, token, ',') && index < 8) {
    try {
      values[index++] = std::stof(token);
    } catch (const std::exception &e) {
      // Handle invalid values, keep as 0.0f
      index++;
    }
  }
  return values;
}

std::array<float, NUM_JOINTS> MachineGait::update(
    const std::array<float, NUM_OBSERVATIONS> &observations) {
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
  auto now = std::chrono::system_clock::now();
  auto now_in_seconds =
      std::chrono::duration<double>(now.time_since_epoch()).count();
  log_line += std::to_string(now_in_seconds) + ",";

  auto &input_buffer = onnx_handler->get_input_buffer();

  // angular velocity
  input_buffer[0] = observations[0] * ANGULAR_VEL_SCALER;
  input_buffer[1] = observations[1] * ANGULAR_VEL_SCALER;
  input_buffer[2] = observations[2] * ANGULAR_VEL_SCALER;

  // projected gravity
  input_buffer[3] = observations[3];
  input_buffer[4] = observations[4];
  input_buffer[5] = observations[5];

  // commands
  input_buffer[6] = observations[7];  // the commands use y as the forward
  input_buffer[7] = observations[6];  // and x as the sideways
  input_buffer[8] = observations[8];

  // joint positions
  for (int i = 0; i < NUM_JOINTS; i++) {
    input_buffer[POSITION_IDX + i] =
        observations[POSITION_IDX + i] - standing_positions[i];
  }

  // joint velocities
  for (int i = 0; i < NUM_JOINTS; i++) {
    input_buffer[VELOCITY_IDX + i] =
        observations[VELOCITY_IDX + i] * VELOCITY_SCALER;
  }

  // previous actions
  for (int i = 0; i < NUM_JOINTS; i++) {
    input_buffer[PREV_ACTION_IDX + i] = previous_actions[i] *
                                        PREV_ACTION_SCALER;
  }

  for (int i = 0; i < NUM_OBSERVATIONS; i++) {
    log_line += std::to_string(input_buffer[i]) + ",";
  }

  onnx_handler->run();

  const auto &output_buffer = onnx_handler->get_output_buffer();

  std::array<float, NUM_JOINTS> positions{};

  for (int i = 0; i < NUM_JOINTS; i++) {
    previous_actions[i] = output_buffer[i];

    const auto delta = output_buffer[i] * ACTION_SCALER;
    positions[i] = standing_positions[i] + delta;

    log_line +=
        std::to_string(positions[i]) + ((i < NUM_JOINTS - 1) ? "," : "");
  }

  Logger::INFO("machine_gait_moving", log_line.c_str());

  // Print standing positions
  // std::cout << "MachineGait::move() - standing positions: ";
  // for (const auto &pos : standing_positions) {
  //   std::cout << pos << " ";
  // }
  // std::cout << std::endl;

  return positions;
}