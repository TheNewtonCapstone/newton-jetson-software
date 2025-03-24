#pragma once

#include "gaits/base_gait.h"
#include <vector>
#include <string>

namespace newton
{
  class WalkingGait : public BaseGait
  {
  private: 
    float frequency;          // Walking frequency in Hz
    float time_step;          // Time step in seconds
    float swing_height;       // Height of leg swing in meters
    float step_length;        // Length of a step in meters
    float stance_force;       // Force for stance legs
    float swing_force;        // Force for swing legs
    float forward_speed;      // Forward speed in m/s
    int steps;                // Step counter for debugging
    int num_legs;             // Number of legs (4 for quadruped)
    float phase;              // Current phase of the gait cycle (0 to 1)
    float leg_phase_duration; // Duration of one leg's phase

    // Gait sequence - the order in which legs are lifted during walking cycle
    std::vector<std::string> leg_sequence;
    
    // Map to store the target positions for each leg in the stance phase
    std::unordered_map<std::string, std::array<float, 3>> stance_pose = {
        {"fl", {0.0f, -0.5, 1.0f}},
        {"fr", {0.0f, 0.5, 1.0f}},
        {"hl", {0.0f, -0.5, 1.0f}},
        {"hr", {0.0f, 0.5, 1.0f}}
    };


  public:
    explicit WalkingGait(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~WalkingGait() = default;

  protected:
    result<void> move() override;
  };
} // namespace newton