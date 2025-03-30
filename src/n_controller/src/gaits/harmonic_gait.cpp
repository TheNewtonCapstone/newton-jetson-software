#include "gaits/harmonic_gait.h"

#include <rclcpp/rclcpp.hpp>

#include "logger.h"

using namespace newton;

HarmonicGait::HarmonicGait(const rclcpp::NodeOptions &options)
    : BaseGait("harmonic_gait", true, options) {
  Logger::get_instance().set_logfile("harmonic_gait.csv");

  std::string log_title = "time,";
  log_title += "hfe_offset,kfe_offset,";
  log_title +=
      "target_fl_hfe,target_fl_kfe,target_fr_hfe,target_fr_kfe,target_hl_hfe,"
      "target_hl_kfe,target_hr_hfe,target_hr_kfe,";
  log_title +=
      "pos_fl_hfe,pos_fl_kfe,pos_fr_hfe,pos_fr_kfe,pos_hl_hfe,pos_hl_kfe,pos_"
      "hr_hfe,pos_hr_kfe,";
  log_title +=
      "vel_fl_hfe,vel_fl_kfe,vel_fr_hfe,vel_fr_kfe,vel_hl_hfe,vel_hl_kfe,vel_"
      "hr_hfe,vel_hr_kfe,";
  log_title +=
      "tor_fl_hfe,tor_fl_kfe,tor_fr_hfe,tor_fr_kfe,tor_hl_hfe,tor_hl_kfe,tor_"
      "hr_hfe,tor_hr_kfe,";
  Logger::INFO("harmonic_gait", log_title.c_str());

  BaseGait::init();
};

std::array<float, NUM_JOINTS> HarmonicGait::update(
    const std::array<float, GaitManager::NUM_OBSERVATIONS> &observations) {
  std::string log_line = "";

  // every 5s, the amplitude will change and after 5 amplitude changes, the
  // frequency will change (up to 5 changes)
  const float INITIAL_AMPLITUDE = 0.5;
  const float INITIAL_FREQUENCY = 0.3;

  // every 5s, the amplitude will change and after 5 amplitude changes, the
  // frequency will change (up to 5 changes)
  static float base_amplitude = INITIAL_AMPLITUDE;
  static float base_frequency = INITIAL_FREQUENCY;
  static int amplitude_changes = 0;
  static int frequency_changes = 0;
  static double last_change_time = 0.0;

  auto now = this->get_clock()->now();
  log_line += std::to_string(now.nanoseconds()) + ",";
  auto current_time = now.seconds();

  // Check if 5 seconds have passed since the last change
  if (current_time - last_change_time >= 60.0) {
    // Update the last change time
    last_change_time = current_time;

    // Change amplitude
    amplitude_changes++;
    base_amplitude += 0.1;

    RCLCPP_INFO(this->get_logger(), "Amplitude changed to %f", base_amplitude);

    // If we've changed amplitude 5 times, change frequency
    if (amplitude_changes >= 5) {
      amplitude_changes = 0;
      base_amplitude = INITIAL_AMPLITUDE;
      frequency_changes++;

      // RCLCPP_INFO(this->get_logger(), "Frequency changed to %f",
      // base_frequency);

      // Increase frequency (up to 5 changes)
      if (frequency_changes < 5) {
        base_frequency += 0.1;
      } else {
        frequency_changes = 0;
        base_frequency = INITIAL_FREQUENCY;
      }
    }
  }

  // Use the updated amplitude and frequency
  float amplitude = base_amplitude;
  float frequency = base_frequency;

  // Simple approach if standing positions are close to 0
  float max_amplitude = M_PI / 2;  // Maximum desired range is from -π/2 to π/2
  float hfe_scaling =
      std::min(1.0f, max_amplitude / amplitude);  // Scale down if needed

  float base_position = amplitude * sin(TWO_PI * frequency * now.seconds());
  float hfe_offset = amplitude * hfe_scaling * base_position;
  float kfe_offset = amplitude * -2.0 * base_position;

  log_line += std::to_string(hfe_offset) + ",";
  log_line += std::to_string(kfe_offset) + ",";

  std::array<float, NUM_JOINTS> positions{};

  // Calculate target positions for all joints
  for (auto &leg : leg_ids) {
    std::string leg_name = leg.first;
    std::array<int, 2> ids = leg.second;

    // Set the target positions
    positions[ids[0]] = leg_standing_positions[leg_name][0] + hfe_offset;
    positions[ids[1]] = leg_standing_positions[leg_name][1] + kfe_offset;
  }

  // Check if test should end
  if (amplitude_changes == 5 && frequency_changes == 5) {
    rclcpp::shutdown();
  }

  // logger
  // Log observations
  for (int i = 0; i < NUM_OBSERVATIONS; i++) {
    log_line += std::to_string(observations[i]) + ",";
  }
  /// log target positions
  for (int i = 0; i < NUM_JOINTS; i++) {
    log_line += std::to_string(positions[i]) + ",";
  }
  // Send target positions to motors
  return positions;

  // Write to log file
  Logger::INFO("harmonic_gait", log_line.c_str());

  return result<void>::success();
}