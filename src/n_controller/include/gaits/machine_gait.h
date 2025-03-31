#pragma once

#include <array>
#include <fstream>
#include <memory>
#include <vector>

#include "gaits/base_gait.h"
#include "handlers/onnx_handler.h"

namespace newton {
class MachineGait : public BaseGait {
 public:
  explicit MachineGait(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MachineGait() = default;

 protected:
  const float ANGULAR_VEL_SCALER = 0.05f;
  const float VELOCITY_SCALER = 0.25f;
  const float PREV_ACTION_SCALER = 1.0f;
  const float ACTION_SCALER = 0.25f;

  std::array<float, NUM_JOINTS> update(
      const std::array<float, NUM_OBSERVATIONS> &observations) override;

 private:
  std::unique_ptr<OnnxHandler> onnx_handler;
  std::array<float, NUM_JOINTS> previous_actions;
  std::array<float, NUM_JOINTS> csv_data{};

  // CSV file handling
  std::ifstream csv_file;
};
}  // namespace newton
