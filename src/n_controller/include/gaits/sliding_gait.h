#pragma once

#include "gaits/base_gait.h"

namespace newton {
class SlidingGait : public BaseGait {
 private:
  float step_length = 0.1;
  float frequency = 1;         // Hz
  float scaling_factor = 0.5;  // scaling factor for the step length

 public:
  explicit SlidingGait(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~SlidingGait() = default;

 protected:
  result<void> move() override;
};
}  // namespace newton
