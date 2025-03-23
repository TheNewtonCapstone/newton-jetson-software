#pragma once

#include "gaits/base_gait.h"

namespace newton
{
  class SlidingGait : public BaseGait
  {
    private:

      float step_length = 0.1;
      float frequency = 1; // Hz
      float scaling_factor = 0.5; // scaling factor for the step length

      std::unordered_map<std::string, float> phases = {
        {"fl", 0.0},
        {"fr", PI},
        {"hl", PI},
        {"hr", 0.0}
      };

      std::unordered_map<std::string, std::array<float, 3>> base_angles = {
        {"fl", {0.0, -0.5, 1.0}},
        {"fr", {0.0, -0.5, 1.0}},
        {"hl", {0.0, -0.5, 1.0}},
        {"hr", {0.0, -0.5, 1.0}}
      };
     
      // map the legs to ids
      std::unordered_map<std::string, std::array<int, 3>> leg_ids = {
        {"fl", {0, 1, 2}},
        {"fr", {3, 4, 5}},
        {"hl", {6, 7, 8}},
        {"hr", {9, 10, 11}}
      };
  public:
    explicit SlidingGait(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~SlidingGait() = default;

  protected:
    result<void> move() override;
  };
} // namespace newton
