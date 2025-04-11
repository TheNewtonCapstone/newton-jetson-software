#pragma once

#include <array>

#include "gaits/base_gait.h"

namespace newton {
class HarmonicGait : public BaseGait {
 public:
  explicit HarmonicGait();
  ~HarmonicGait() = default;

 protected:
  std::array<float, NUM_JOINTS> update(
      const std::array<float, NUM_OBSERVATIONS> &observations) override;
};
}  // namespace newton
