#include "gaits/harmonic_gait.h"
#include "logger.h"

#include <rclcpp/rclcpp.hpp>

using namespace newton;

HarmonicGait::HarmonicGait(const rclcpp::NodeOptions &options)
    : BaseGait("harmonic_gait", true, options)
{
  
  Logger::get_instance().set_logfile("harmonic_gait.csv");

  std::string log_title ="time,";
  log_title += "hfe_offset,kfe_offset,"; 
  log_title += "pos_fl_hfe, pos_fl_kfe, pos_fr_hfe, pos_fr_kfe, pos_hl_hfe, pos_hl_kfe, pos_hr_hfe, pos_hr_kfe,";
  log_title += "vel_fl_hfe, vel_fl_kfe, vel_fr_hfe, vel_fr_kfe, vel_hl_hfe, vel_hl_kfe, vel_hr_hfe, vel_hr_kfe,";
  log_title += "tor_fl_hfe, tor_fl_kfe, tor_fr_hfe, tor_fr_kfe, tor_hl_hfe, tor_hl_kfe, tor_hr_hfe, tor_hr_kfe,";
  Logger::INFO("harmonic_gait", log_title.c_str());

  BaseGait::init();
};

result<void> HarmonicGait::move()
{
  std::string log_line = "";


  // every 5s, the amplitude will change and after 5 amplitude changes, the frequency will change (up to 5 changes)

  const float INITIAL_AMPLITUDE = 0.5;
  const float INITIAL_FREQUENCY = 0.3;

  // every 5s, the amplitude will change and after 5 amplitude changes, the frequency will change (up to 5 changes)
  static float base_amplitude = INITIAL_AMPLITUDE;
  static float base_frequency = INITIAL_FREQUENCY;
  static int amplitude_changes = 0;
  static int frequency_changes = 0;
  static double last_change_time = 0.0;

  auto now = this->get_clock()->now();
  log_line += std::to_string(now.nanoseconds()) + ",";
  auto current_time = now.seconds();

  // Check if 5 seconds have passed since the last change
  if (current_time - last_change_time >= 3.0)
  {
    // Update the last change time
    last_change_time = current_time;

    // Change amplitude
    amplitude_changes++;
    base_amplitude += 0.1;

    RCLCPP_INFO(this->get_logger(), "Amplitude changed to %f", base_amplitude);

    // If we've changed amplitude 5 times, change frequency
    if (amplitude_changes >= 5)
    {
      amplitude_changes = 0;
      base_amplitude = INITIAL_AMPLITUDE;
      frequency_changes++;

      // RCLCPP_INFO(this->get_logger(), "Frequency changed to %f", base_frequency);

      // Increase frequency (up to 5 changes)
      if (frequency_changes < 5)
      {
        base_frequency += 0.1;
      }
      else
      {
        frequency_changes = 0;
        base_frequency = INITIAL_FREQUENCY;
      }
    }
  }

  // Use the updated amplitude and frequency
  float amplitude = base_amplitude;
  float frequency = base_frequency;

  // std::array<float, 33> input_buffer{};

  // // angular velocity
  // const auto angular_velocity_scaler = 0.25;
  // input_buffer[0] = imu->angular_velocity.x * angular_velocity_scaler;
  // input_buffer[1] = imu->angular_velocity.y * angular_velocity_scaler;
  // input_buffer[2] = imu->angular_velocity.z * angular_velocity_scaler;

  // // projected gravity
  // input_buffer[3] = imu->projected_gravity.x;
  // input_buffer[4] = imu->projected_gravity.y;
  // input_buffer[5] = imu->projected_gravity.z;

  // // commands
  // // input_buffer[6] = cmd->linear_velocity.x;
  // // input_buffer[7] = cmd->linear_velocity.y;
  // // input_buffer[8] = cmd->angular_velocity.z;

  // input_buffer[6] = 1.0;
  // input_buffer[7] = 0.0;
  // input_buffer[8] = 0.0;

  // joint positions
  // for (int i = 0; i < NUM_JOINTS; i++)
  // {
  //   input_buffer[9 + i] = joints[i].curr_pos - standing_positions[i];
  // }

  // joint velocities
  // const auto joint_velocity_scaler = 0.05;
  // for (int i = 0; i < NUM_JOINTS; i++)
  // {
  //   input_buffer[17 + i] = joints[i].curr_vel * joint_velocity_scaler;
  // }

  // for (int i = 0; i < 33; i++)
  // {
  //   log_line += std::to_string(input_buffer[i]) + ",";
  // }

  // get the offsets
  float base_position = INITIAL_AMPLITUDE *  sin(TWO_PI * INITIAL_FREQUENCY * now.seconds());
  // float haa_offset = amplitude * 0.5 * base_position;
  float hfe_offset = INITIAL_AMPLITUDE * 1.0 * base_position;
  float kfe_offset = INITIAL_AMPLITUDE * -2.0 * base_position;

  log_line += std::to_string(hfe_offset) + ",";
  log_line += std::to_string(kfe_offset) + ",";

  std::array<float, NUM_JOINTS> positions{};
  
  int counter = 0; 
  for (auto &leg : leg_ids)
  {
    // Get the phase

    std::string leg_name = leg.first;
    std::array<int, 2> ids = leg.second;

    // set the positions, only the HFE joints are affected
    // Logger::INFO("harmonic_gait", "Leg: %s, id_1:%d, id_2: %d:HFE offset: %f, KFE offset: %f", leg_name.c_str(), ids[0], ids[1],  hfe_offset, kfe_offset);
    // positions[counter++] = leg_standing_positions[leg_name][0] + hfe_offset;
    // positions[counter++] = leg_standing_positions[leg_name][1] + kfe_offset;
    positions[ids[0]] = leg_standing_positions[leg_name][0] + hfe_offset;
    positions[ids[1]] = leg_standing_positions[leg_name][1] + kfe_offset;

    // // log the positions
    // log_line += std::to_string(positions[ids[0]]) + ",";
    // log_line += std::to_string(positions[ids[1]]) + ",";
  }

    // RCLCPP_INFO(this->get_logger(), "Amplitude and frequency changes completed");
  if (amplitude_changes == 5 && frequency_changes == 5)
  {
    rclcpp::shutdown();
  }

  // log the positions
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    log_line += std::to_string(joints[i].curr_pos) + ",";
  }

  // log the velocities
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    log_line += std::to_string(joints[i].curr_vel) + ",";
  }

  // log the torques
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    log_line += std::to_string(joints[i].curr_torque) + ",";
  }

  set_joints_position(positions);

  Logger::INFO("harmonic_gait", log_line.c_str());

  return result<void>::success();
}
