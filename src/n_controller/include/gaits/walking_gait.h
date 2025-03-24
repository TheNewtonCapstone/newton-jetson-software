#pragma once

#include "gaits/base_gait.h"

namespace newton
{
  class WalkingGait : public BaseGait
  {
  private: 
        float frequency, time_step, swing_height, step_length, stance_force, swing_force, forward_speed;
        int steps, num_legs, phase;

        float leg_phase_duration;

  public:
    explicit WalkingGait(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~WalkingGait() = default;

  protected:
    result<void> move() override;
  };
} // namespace newton
