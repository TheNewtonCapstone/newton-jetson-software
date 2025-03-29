#pragma once

#include "gaits/base_gait.h"
#include "handlers/onnx_handler.h"
#include <memory>
#include <array>
#include <fstream>
#include <vector>

namespace newton
{
  class MachineGait : public BaseGait
  {
  public:
    explicit MachineGait(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~MachineGait() = default;

  protected:
    result<void> move() override;

  private:
    std::array<float, NUM_JOINTS> previous_actions;
    std::array<float, NUM_JOINTS> csv_data{}; // Stores the current line of CSV data (joint positions + 1 for time)
    std::unique_ptr<OnnxHandler> onnx_handler;

    // CSV file handling
    std::ifstream csv_file;
  };
} // namespace newton
