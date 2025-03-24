#include "gaits/walking_gait.h"
#include "math.h"
#include "logger.h"

#include <rclcpp/rclcpp.hpp>
#include <vector>

using namespace newton;

WalkingGait::WalkingGait(const rclcpp::NodeOptions &options)
    : BaseGait("walking_gait", true, options) 
{
    Logger::INFO("walking_gait", "Initializing walking gait controller");
    
    // Initialize parameters
    this->phase = 0.0f;
    this->frequency = 0.5f;      // Walking frequency in Hz
    this->time_step = 0.02f;     // 20ms = 0.02s (from the timer in BaseGait)
    this->swing_height = 0.1f;
    this->step_length = 0.05f;
    this->stance_force = 10.0f;
    this->swing_force = 5.0f;
    this->forward_speed = 0.1f;
    this->steps = 0;
    this->num_legs = 4;
    this->leg_phase_duration = 1.0f / num_legs;
    
    // Define leg sequence for walking gait - order matters for coordination
    leg_sequence = {"fl", "hr", "fr", "hl"};
    
    // Initialize stance pose from base_gait's leg_standing_positions
    stance_pose = leg_standing_positions;
    
    BaseGait::init();
};

result<void> WalkingGait::move()
{
    // Update the phase 
    phase = fmod(phase + time_step * frequency, 1.0f);
    
    // Create a fresh copy of the stance pose for this iteration
    auto pose = stance_pose;
    
    // Determine which leg is in swing phase
    // Each leg gets 1/4 of the cycle with 25% duty factor
    int swing_leg_idx = static_cast<int>(phase / leg_phase_duration);
    std::string swing_leg = leg_sequence[swing_leg_idx];
    
    // Calculate normalized phase for the current swing leg
    float swing_phase = (phase - swing_leg_idx * leg_phase_duration) / leg_phase_duration;
    
    // Identify which legs are in stance phase (all except the swing leg)
    std::vector<std::string> stance_legs;
    for (const auto& leg : leg_sequence) {
        if (leg != swing_leg) {
            stance_legs.push_back(leg);
        }
    }
    
    // Apply leg motions for STANCE legs - provide propulsion and stability
    for (const auto& leg : stance_legs) {
        float direction;
        
        // In stance phase, legs should push backward to propel robot forward
        // For front legs: negative hip adjustment pushes backward
        // For hind legs: positive hip adjustment pushes backward
        if (leg.substr(0, 1) == "f") {  // Front legs
            direction = -1.0f;  // Push backward
        } else {  // Hind legs
            direction = 1.0f;   // Push backward
        }
        
        // Create a backward-pushing motion during stance
        float hip_stance_adjust = direction * step_length * 0.5f;  // Consistent push
        
        // Apply the stance motion (push backward) - adjust hip joint (index 1)
        pose[leg][1] = stance_pose[leg][1] + hip_stance_adjust;
        
        // Apply slight downward pressure during stance for better grip - adjust knee joint (index 2)
        pose[leg][2] = stance_pose[leg][2] - 0.05f;
    }
    
    // Apply leg motion for the SWING leg
    // For the height (knee) motion, use a parabolic trajectory
    float height_factor = 4.0f * swing_phase * (1.0f - swing_phase);  // Parabolic curve: 0->1->0
    float lift_amount = swing_height * height_factor;
    
    float direction;
    // For front/back (hip) motion, we want a smooth trajectory
    if (swing_leg.substr(0, 1) == "f") {  // Front legs
        direction = -1.0f;  // Moving from back to front
    } else {  // Hind legs
        direction = 1.0f;   // Moving from front to back
    }
    
    // Create a smooth forward motion during swing - linear interpolation
    float forward_factor = swing_phase;
    float hip_swing_adjust = direction * step_length * (1.0f - 2.0f * forward_factor);
    
    // Apply the swing trajectory to the current swing leg
    pose[swing_leg][1] = stance_pose[swing_leg][1] + hip_swing_adjust;
    pose[swing_leg][2] = stance_pose[swing_leg][2] + lift_amount;
    
    // Prepare final joint positions array
    std::array<float, NUM_JOINTS> final_positions;
    
    // Start with standing positions as default
    final_positions = standing_positions;
    
    // Populate the array with joint positions from the pose map
    // Convert from leg-centric structure to flat array
    for (const auto& [leg_name, leg_indices] : leg_ids) {
        if (pose.find(leg_name) != pose.end()) {
            for (size_t i = 0; i < 3; ++i) {
                final_positions[leg_indices[i]] = pose[leg_name][i];
            }
        }
    }
    
    // Send joint positions to servos
    set_joints_position(final_positions);
    
    // Debug logging
    steps++;
    if (steps % 50 == 0) {
      for(const auto& pose : final_positions) {
        Logger::INFO("walking_gait", "Joint position: %d", pose);
      }

      // log everything
      for (const auto& [leg_name, leg_indices] : leg_ids) {
        Logger::INFO("walking_gait", "Leg %s: %.2f, %.2f, %.2f", leg_name, pose[leg_name][0], pose[leg_name][1], pose[leg_name][2]);
      }
    }
    
    return result<void>::success();
}