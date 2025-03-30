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
  std::array<float, GaitManager::NUM_JOINTS> update(
      const std::array<float, GaitManager::NUM_OBSERVATIONS> &observations);

 private:
  static constexpr float ANGULAR_VEL_SCALER = 0.05f;
  static constexpr float VELOCITY_SCALER = 0.25f;
  static constexpr float PREV_ACTION_SCALER = 1.0f;
  static constexpr float ACTION_SCALER = 0.25f;

  std::array<float, NUM_JOINTS> previous_actions;
  std::array<float, NUM_JOINTS>
      csv_data{};  // Stores the current line of CSV data (joint positions + 2
                   // for time)
  std::unique_ptr<OnnxHandler> onnx_handler;

  // CSV file handling
  std::ifstream csv_file;
};
}  // namespace newton
