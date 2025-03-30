#pragma once

#include "gaits/base_gait.h"

namespace newton {
class StandingGait : public BaseGait {
 public:
  explicit StandingGait(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~StandingGait() = default;

 protected:
  std::array<float, GaitManager::NUM_JOINTS> update(
      const std::array<float, GaitManager::NUM_OBSERVATIONS> &observations);
};
}  // namespace newton
