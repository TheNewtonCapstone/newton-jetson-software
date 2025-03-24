#include "gaits/sliding_gait.h"

#include <rclcpp/rclcpp.hpp>

#include "logger.h"
#include "math.h"
#include "unit.h"

using namespace newton;

SlidingGait::SlidingGait(const rclcpp::NodeOptions &options)
    : BaseGait("harmonic_gait", false, options) {
  BaseGait::init();
  //   Declare step length parameter
  this->declare_parameter("step_length", 0.1);
  this->get_parameter("step_length", step_length);
  Logger::INFO("SM", "Step length: %f", step_length);
};

result<void> SlidingGait::move() {
  const float FREQUENCY = 0.5;

  auto now = this->get_clock()->now();

  float hfe_offset = 0.0;
  std::array<float, NUM_JOINTS> positions{};

  // loop throug the leg ids
  for (auto &leg : leg_ids) {
    // Get the phase
    std::string leg_name = leg.first;
    std::array<int, 3> ids = leg.second;

    float phase = phases[leg_name];
    float cycle_pos = sin(TWO_PI * FREQUENCY * now.seconds() + phase);

    hfe_offset = step_length * cycle_pos;

    // set the positions, only the HFE joints are affected
    positions[ids[0]] = leg_standing_positions[leg_name][0];
    positions[ids[1]] = leg_standing_positions[leg_name][1] + hfe_offset;
    positions[ids[2]] = leg_standing_positions[leg_name][2];
    // Logger::DEBUG("SM", "Sliding motion %ld, HFE offset: %f", now.nanoseconds(), hfe_offset);
  }

  set_joints_position(positions);

  return result<void>::success();
}
