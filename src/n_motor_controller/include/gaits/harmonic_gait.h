#pragma once

#include "gaits/base_gait.h"

namespace newton
{
  class HarmonicGait : public BaseGait
  {
  public:
    explicit HarmonicGait(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~HarmonicGait() = default;

  protected:
    void move() override;
 };
} // namespace newton
