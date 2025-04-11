#include "gaits/standing_gait.h"

#include <cmath>

#include "logger.h"
#include "unit.h"

using namespace newton;

StandingGait::StandingGait() : BaseGait() {
  Logger::get_instance().set_logfile("standing.csv");

  std::string log_title = "standing_gait,";

  log_title += "time,";
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
};

std::array<float, NUM_JOINTS> StandingGait::update(
    const std::array<float, NUM_OBSERVATIONS> &observations) {
  std::string log_line = "";

  // every 5s, the amplitude will change and after 5 amplitude changes, the
  // frequency will change (up to 5 changes)
  const float INITIAL_AMPLITUDE = 0.1;
  const float INITIAL_FREQUENCY = 0;

  // every 5s, the amplitude will change and after 5 amplitude changes, the
  // frequency will change (up to 5 changes)
  static float base_amplitude = INITIAL_AMPLITUDE;
  static float base_frequency = INITIAL_FREQUENCY;
  static double last_change_time = 0.0;

  auto now = std::chrono::system_clock::now();
  auto now_in_seconds= std::chrono::duration<double>(now.time_since_epoch()).count();
  log_line += std::to_string(now_in_seconds) + ",";

  // Use the updated amplitude and frequency
  float amplitude = base_amplitude;
  float frequency = base_frequency;

  // Simple approach if standing positions are close to 0
  float max_amplitude = M_PI / 2;  // Maximum desired range is from -π/2 to π/2
  float hfe_scaling =
      std::min(1.0f, max_amplitude / amplitude);  // Scale down if needed

  float base_position =
      INITIAL_AMPLITUDE * sin(TWO_PI * INITIAL_FREQUENCY * now_in_seconds);
  float hfe_offset = INITIAL_AMPLITUDE * hfe_scaling * base_position;
  float kfe_offset = INITIAL_AMPLITUDE * -2.0 * base_position;

  log_line += std::to_string(hfe_offset) + ",";
  log_line += std::to_string(kfe_offset) + ",";

  std::array<float, NUM_JOINTS> positions{};

  // Calculate target positions for all joints
  for (int i = 0; i < NUM_JOINTS; i++) {
    if (i > NUM_JOINTS / 2) {
      positions[i] = standing_positions[i] + kfe_offset +
                     sin(TWO_PI * INITIAL_FREQUENCY * now_in_seconds) *
                         base_amplitude;  // Add sine wave to the offset
      continue;
    }
    if (i % 2 == 0) {
      // HFE joint
      positions[i] = standing_positions[i] + hfe_offset;
    } else {
      // KFE joint
      positions[i] = standing_positions[i] + kfe_offset;
    }
  }

  // log observations
  for (int i = 0; i < NUM_OBSERVATIONS; i++) {
    log_line += std::to_string(observations[i]) + ",";
  }

  // Log target positions
  for (int i = 0; i < NUM_JOINTS; i++) {
    log_line += std::to_string(positions[i]) + ",";
  }

  Logger::INFO("standing_gait", log_line.c_str());
  return positions;
}