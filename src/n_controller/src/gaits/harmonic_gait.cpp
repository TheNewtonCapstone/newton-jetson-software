#include "gaits/harmonic_gait.h"
#include "logger.h"

#include <rclcpp/rclcpp.hpp>

using namespace newton;

HarmonicGait::HarmonicGait(const rclcpp::NodeOptions &options)
    : BaseGait("harmonic_gait", true, options)
{
  Logger::get_instance().set_logfile("harmonic_gait.log");
  Logger::INFO("harmonic_gait", "time,angular_velocity_x,angular_velocity_y,angular_velocity_z,projected_gravity_x,projected_gravity_y,projected_gravity_z,linear_velocity_x,linear_velocity_y,angular_velocity_z,"
                                "joint_delta_1,joint_delta_2,joint_delta_3,joint_delta_4,joint_delta_5,joint_delta_6,joint_delta_7,joint_delta_8,joint_delta_9,joint_delta_10,joint_delta_11,joint_delta_12,"
                                "joint_velocity_1,joint_velocity_2,joint_velocity_3,joint_velocity_4,joint_velocity_5,joint_velocity_6,joint_velocity_7,joint_velocity_8,joint_velocity_9,joint_velocity_10,joint_velocity_11,joint_velocity_12"
                                "action_1,action_2,action_3,action_4,action_5,action_6,action_7,action_8,action_9,action_10,action_11,action_12");

  BaseGait::init();
};

result<void> HarmonicGait::move()
{
  std::string log_line = "";

  // every 5s, the amplitude will change and after 5 amplitude changes, the frequency will change (up to 5 changes)

  const float INITIAL_AMPLITUDE = 0.5;
  const float INITIAL_FREQUENCY = 0.1;

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
  float base_position = amplitude * sin(TWO_PI * frequency * now.seconds());
  float haa_offset = amplitude * 0.5 * base_position;
  float hfe_offset = amplitude * 1.0 * base_position;
  float kfe_offset = amplitude * -2.0 * base_position;

  std::array<float, NUM_JOINTS> positions{};
  for (auto &leg : leg_ids)
  {
    // Get the phase
    std::string leg_name = leg.first;
    std::array<int, 2> ids = leg.second;

    // set the positions, only the HFE joints are affected
    positions[ids[0]] = leg_standing_positions[leg_name][0] + hfe_offset;
    positions[ids[1]] = leg_standing_positions[leg_name][1] + kfe_offset;

    // log the positions
    log_line += std::to_string(positions[ids[0]]) + ",";
    log_line += std::to_string(positions[ids[1]]) + ",";
  }

  if (amplitude_changes == 5 && frequency_changes == 5)
  {
    RCLCPP_INFO(this->get_logger(), "Amplitude and frequency changes completed");
    rclcpp::shutdown();
  }

  // set_joints_position(positions);

  Logger::INFO("harmonic_gait", log_line.c_str());

  return result<void>::success();
}
