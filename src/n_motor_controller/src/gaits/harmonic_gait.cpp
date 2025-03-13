#include "gaits/harmonic_gait.h"
#include "logger.h"

#include <rclcpp/rclcpp.hpp>

using namespace newton;
using namespace std::chrono_literals;

HarmonicGait::HarmonicGait(const rclcpp::NodeOptions &options)
    : BaseGait("harmonic_gait", options)
{
  BaseGait::init();
};

void HarmonicGait::move()
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

  Logger::INFO("HarmonicGait", "Waiting for encoder offsets to be loaded");
  while (!offset_loaded)
  {
  };

  std::array<float, NUM_JOINTS> jp_mult = {
      1, 1, -1.0,
      1, -1.0, 1.0,
      1, 1, -1,
      1, -1.0, 1.0,
  };

  std::array<float, NUM_JOINTS> standing_positions = {
      0.0, 0.5, -1.0, // fl
      0.0, 0.5, 1.0,  // fr
      0.0, 0.5, 1.0,  // hl
      0.0, 0.5, 1.0,  // hr
  };

  // get the offsets
  float base_position = AMPLITUDE * sin(TWO_PI * FREQUENCY * now.seconds());
  float hfe_offset = AMPLITUDE * -1.0 * base_position;
  float kfe_offset = AMPLITUDE * 2.0 * base_position;

  std::array<double, NUM_JOINTS> positions{};
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    if (i % 3 == 1)
    {
      positions[i] = standing_positions[i] + (hfe_offset * jp_mult[i]);
    }
    else if (i % 3 == 2)
    {
      positions[i] = standing_positions[i] + (kfe_offset * jp_mult[i]);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Harmonic motion %ld, Base position: %f, HFE offset: %f, KFE offset: %f", now.nanoseconds(), base_position, hfe_offset, kfe_offset);

  // print the positions
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    RCLCPP_INFO(this->get_logger(), "Joint %d: %f", i, positions[i]);
  }

  RCLCPP_INFO(this->get_logger(), "Duration: %f", duration);

  for (int i = 0; i < NUM_JOINTS; i++)
  {
    set_joint_position(standing_positions[i], i);
  }
}