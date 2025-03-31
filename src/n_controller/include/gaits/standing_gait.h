#pragma once

#include <array>

#include "gaits/base_gait.h"

namespace newton {
class StandingGait : public BaseGait {
 public:
  explicit StandingGait();
  ~StandingGait() = default;

 protected:
  std::array<float, NUM_JOINTS> update(
      const std::array<float, NUM_OBSERVATIONS> &observations);
};
}  // namespace newton
