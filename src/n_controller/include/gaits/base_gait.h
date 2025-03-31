#pragma once

#include "globals.h"

namespace newton {
class BaseGait {
 public:
  explicit BaseGait();
  ~BaseGait() = default;

  virtual std::array<float, NUM_JOINTS> update(
      const std::array<float, NUM_OBSERVATIONS> &observations) = 0;
};
}  // namespace newton
