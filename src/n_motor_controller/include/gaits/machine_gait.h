#pragma once

#include "gaits/base_gait.h"
#include "handlers/onnx_handler.h"

namespace newton
{
    class MachineGait : public BaseGait
    {
    public:
        explicit MachineGait(const std::string model_path, const int num_inputs = 45, const int num_outputs = 12,
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
        ~MachineGait() = default;

    protected:
        void move() override;

    private:
        std::array<float, NUM_JOINTS> previous_actions;
        std::unique_ptr<OnnxHandler> onnx_handler;
   };
} // namespace newton
