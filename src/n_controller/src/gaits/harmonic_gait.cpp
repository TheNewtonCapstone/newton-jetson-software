#include "gaits/harmonic_gait.h"
#include "logger.h"

#include <rclcpp/rclcpp.hpp>

using namespace newton;

HarmonicGait::HarmonicGait(const rclcpp::NodeOptions &options)
    : BaseGait("harmonic_gait", true, options)
{
  Logger::get_instance().set_logfile("harmonic_gait.log");
  std::string log_title ="time,";
  log_title += "ang_vel_x, ang_vel_y, ang_vel_z,";
  log_title += "proj_grav_x, proj_grav_y, proj_grav_z,";
  log_title += "cmd_lin_vel_x, cmd_lin_vel_y, cmd_ang_vel_z,";
  log_title += "fl_hfe, fl_kfe, fr_hfe, fr_kfe, hl_hfe, hl_kfe, hr_hfe, hr_kfe,";
  log_title += "vel_fl_hfe, vel_fl_kfe, vel_fr_hfe, vel_fr_kfe, vel_hl_hfe, vel_hl_kfe, vel_hr_hfe, vel_hr_kfe,";
  log_title += "act_fl_hfe, act_fl_kfe, act_fr_hfe, act_fr_kfe, act_hl_hfe, act_hl_kfe, act_hr_hfe, act_hr_kfe,";


  BaseGait::init();
};

result<void> HarmonicGait::move()
{
  std::string log_line = "";

  // every 5s, the amplitude will change and after 5 amplitude changes, the frequency will change (up to 5 changes)

  const float INITIAL_AMPLITUDE = 0.5;
  const float INITIAL_FREQUENCY = 0.2;

  // every 5s, the amplitude will change and after 5 amplitude changes, the frequency will change (up to 5 changes)
  static float base_amplitude = INITIAL_AMPLITUDE;
  static float base_frequency = INITIAL_FREQUENCY;
  static int amplitude_changes = 0;
  static int frequency_changes = 0;
  static double last_change_time = 0.0;

  auto now = this->get_clock()->now();
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

      RCLCPP_INFO(this->get_logger(), "Frequency changed to %f", base_frequency);

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

  std::array<float, 33> input_buffer{};

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
    input_buffer[21 + i] = joints[i].curr_vel * joint_velocity_scaler;
  }

  for (int i = 0; i < 33; i++)
  {
    log_line += std::to_string(input_buffer[i]) + ",";
  }

  // get the offsets
  float base_position = INITIAL_AMPLITUDE * sin(TWO_PI * frequency * now.seconds());
  // float haa_offset = amplitude * 0.5 * base_position;
  float hfe_offset = INITIAL_AMPLITUDE * 1.0 * base_position;
  float kfe_offset = INITIAL_AMPLITUDE * -2.0 * base_position;

  std::array<float, NUM_JOINTS> positions{};
  
  int counter = 0; 
  for (auto &leg : leg_ids)
  {
    // Get the phase

    std::string leg_name = leg.first;
    std::array<int, 2> ids = leg.second;

    // set the positions, only the HFE joints are affected
    Logger::INFO("harmonic_gait", "Leg: %s, id_1:%d, id_2: %d:HFE offset: %f, KFE offset: %f", leg_name.c_str(), ids[0], ids[1],  hfe_offset, kfe_offset);
    positions[counter++] = leg_standing_positions[leg_name][0] + hfe_offset;
    positions[counter++] = leg_standing_positions[leg_name][1] + kfe_offset;
    // positions[ids[0]] = leg_standing_positions[leg_name][0] + hfe_offset;
    // positions[ids[1]] = leg_standing_positions[leg_name][1] + kfe_offset;

    // // log the positions
    // log_line += std::to_string(positions[ids[0]]) + ",";
    // log_line += std::to_string(positions[ids[1]]) + ",";
  }

    RCLCPP_INFO(this->get_logger(), "Amplitude and frequency changes completed");
  // if (amplitude_changes == 5 && frequency_changes == 5)
  // {
  //   rclcpp::shutdown();
  // }

  set_joints_position(positions);

  Logger::INFO("harmonic_gait", log_line.c_str());
  Logger::INFO("harmonic_gait", "Sent joint positions");

  return result<void>::success();
}
