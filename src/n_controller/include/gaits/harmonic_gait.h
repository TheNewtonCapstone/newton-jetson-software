#pragma once

#include "gaits/base_gait.h"

namespace newton
{
  class SlidingGait : public BaseGait
  {
  public:
    explicit SlidingGait(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~SlidingGait() = default;

  protected:
    result<void> move() override;
  };
} // namespace newton
