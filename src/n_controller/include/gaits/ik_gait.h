#pragma once

#include "gaits/base_gait.h"
#include "handlers/ik_handler.h"

namespace newton {
class IkGait : public BaseGait {
 public:
  explicit IkGait(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~IkGait() = default;

 protected:
  result<void> move() override;

 private:
  IkSolver m_ik_solver;
};
}  // namespace newton
