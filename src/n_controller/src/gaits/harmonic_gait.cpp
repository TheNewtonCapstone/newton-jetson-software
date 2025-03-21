#include "gaits/harmonic_gait.h"
#include "logger.h"

#include <rclcpp/rclcpp.hpp>

using namespace newton;

HarmonicGait::HarmonicGait(const rclcpp::NodeOptions &options)
    : BaseGait("harmonic_gait", options)
{
  BaseGait::init();
};

result<void> HarmonicGait::move()
{
  const float MAX_DURATION = 30.0;
  const float AMPLITUDE = 0.5;
  const float FREQUENCY = 0.5;
  const float TWO_PI = 6.28318530718;

  auto now = this->get_clock()->now();
  auto duration = (now - last_time).seconds();

  if (duration > MAX_DURATION)
  {
    RCLCPP_INFO(this->get_logger(), "Finished Harmonic Gait");
    shutdown();
  }

  // get the offsets
  float base_position = AMPLITUDE * sin(TWO_PI * FREQUENCY * now.seconds());
  float hfe_offset = AMPLITUDE * -1.0 * base_position;
  float kfe_offset = AMPLITUDE * 2.0 * base_position;

  std::array<float, NUM_JOINTS> positions{};
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    if (i % 3 == 1)
    {
      positions[i] = standing_positions[i] + (hfe_offset * direction_mult[i]);
    }
    else if (i % 3 == 2)
    {
      positions[i] = standing_positions[i] + (kfe_offset * direction_mult[i]);
    }
  }

  RCLCPP_INFO(
      this->get_logger(),
      "Harmonic motion %ld, Base position: %f, HFE offset: %f, KFE offset: %f",
      now.nanoseconds(), base_position, hfe_offset, kfe_offset);

  RCLCPP_INFO(this->get_logger(), "Duration: %f", duration);

  set_joints_position(positions);
}
